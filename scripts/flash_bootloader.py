#!/usr/bin/env python3
"""
Bootloader Update Tool for Mainboard Firmware
Uses admin CAN channel (0x3f0/0x3f1) to flash bootloader over CAN

Copyright (C) 2026
This file may be distributed under the terms of the GNU GPLv3 license.
"""
import sys
import os
import socket
import struct
import asyncio
import argparse
import pathlib
from typing import Optional
import zlib

# CAN frame format: <IB3x8s = ID (4 bytes), length (1 byte), padding (3 bytes), data (8 bytes)
CAN_FMT = "<IB3x8s"

# Admin channel IDs
CANBUS_ID_ADMIN = 0x3f0
CANBUS_ID_ADMIN_RESP = 0x3f1

# Admin command codes
ADMIN_CMD_BOOTLOADER_UPDATE_START = 0x20
ADMIN_CMD_BOOTLOADER_UPDATE_CHUNK = 0x21
ADMIN_CMD_BOOTLOADER_UPDATE_VERIFY = 0x22
ADMIN_CMD_BOOTLOADER_UPDATE_COMMIT = 0x23
ADMIN_CMD_BOOTLOADER_UPDATE_ABORT = 0x24

# Response codes
ADMIN_RESP_SUCCESS = 0x00
ADMIN_RESP_ERROR = 0x01
ADMIN_RESP_BUSY = 0x02
ADMIN_RESP_READY = 0x03

# Constants
BOOTLOADER_MAX_SIZE = 0x2000  # 8KB
CHUNK_SIZE = 5  # 5 bytes per CAN message
FLASH_PAGE_SIZE = 0x800  # 2KB pages on STM32F072


def output(msg: str) -> None:
    """Print message with flush"""
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()


def crc32(data: bytes) -> int:
    """Calculate CRC32 of data"""
    return zlib.crc32(data) & 0xFFFFFFFF


class BootloaderFlasher:
    def __init__(self, can_interface: str, uuid: str, bootloader_path: pathlib.Path):
        self.can_interface = can_interface
        self.uuid = bytes.fromhex(uuid)
        if len(self.uuid) != 6:
            raise ValueError("UUID must be 12 hex characters (6 bytes)")
        self.bootloader_path = bootloader_path
        self.cansock: Optional[socket.socket] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.response_queue: asyncio.Queue = asyncio.Queue()
        
    async def _send_admin_command(self, cmd: int, payload: bytes = b"") -> None:
        """Send admin command with UUID targeting"""
        # Admin message format: [CMD (1 byte)] [payload bytes...]
        msg_data = bytes([cmd]) + payload
        
        # Pad to 8 bytes if needed
        if len(msg_data) < 8:
            msg_data += b'\x00' * (8 - len(msg_data))
        elif len(msg_data) > 8:
            raise ValueError(f"Admin command too long: {len(msg_data)} > 8")
        
        # Pack CAN frame
        packet = struct.pack(CAN_FMT, CANBUS_ID_ADMIN, len(msg_data), msg_data)
        await self.loop.sock_sendall(self.cansock, packet)
        
    def _handle_can_response(self) -> None:
        """Handle incoming CAN messages"""
        try:
            data = self.cansock.recv(4096)
        except socket.error:
            return
            
        if not data:
            return
            
        # Process all complete frames in buffer
        offset = 0
        while offset + 16 <= len(data):
            packet = data[offset:offset + 16]
            can_id, length, frame_data = struct.unpack(CAN_FMT, packet)
            can_id &= socket.CAN_EFF_MASK
            
            if can_id == CANBUS_ID_ADMIN_RESP:
                payload = frame_data[:length]
                # Response format: [CMD_ECHO] [data...]
                # We just queue the entire payload
                self.response_queue.put_nowait(payload)
                    
            offset += 16
    
    async def _wait_response(self, timeout: float = 2.0,
                              expected_cmd: Optional[int] = None) -> bytes:
        """Wait for admin response, optionally filtering by command code.
        
        When expected_cmd is set, any responses with a different command echo
        byte (e.g. stale chunk ACKs) are silently discarded.
        """
        deadline = asyncio.get_event_loop().time() + timeout
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                raise TimeoutError("No response from device")
            try:
                resp = await asyncio.wait_for(
                    self.response_queue.get(), min(remaining, timeout))
            except asyncio.TimeoutError:
                raise TimeoutError("No response from device")
            if expected_cmd is None or (len(resp) > 0 and resp[0] == expected_cmd):
                return resp
            # else: stale response for a different command — discard and retry
    
    async def start_update(self, size: int) -> None:
        """Send START command with UUID and size in KB"""
        size_kb = (size + 1023) // 1024  # Round up to KB
        output(f"Starting bootloader update: {size} bytes ({size_kb} KB)")
        # Payload: UUID (6 bytes) + size_kb (1 byte)
        payload = self.uuid + bytes([size_kb])
        await self._send_admin_command(ADMIN_CMD_BOOTLOADER_UPDATE_START, payload)
        
        # Wait for response (we'll get chunk size and staging address back)
        try:
            resp_data = await self._wait_response(
                timeout=3.0, expected_cmd=ADMIN_CMD_BOOTLOADER_UPDATE_START)
            output("Device ready to receive bootloader data")
        except TimeoutError:
            output("Warning: No response to START, continuing anyway...")
    
    async def send_chunk(self, offset: int, data: bytes) -> None:
        """Send a data chunk (5 bytes) with offset"""
        if len(data) > CHUNK_SIZE:
            raise ValueError(f"Chunk too large: {len(data)} > {CHUNK_SIZE}")
        
        # Pad to 5 bytes if needed
        if len(data) < CHUNK_SIZE:
            data = data + b'\x00' * (CHUNK_SIZE - len(data))
        
        # Payload: offset (2 bytes, little endian) + data (5 bytes)
        payload = struct.pack("<H", offset) + data
        await self._send_admin_command(ADMIN_CMD_BOOTLOADER_UPDATE_CHUNK, payload)
        
        # Small delay for flow control (prevent socket buffer overflow)
        await asyncio.sleep(0.001)  # 1ms delay
    
    async def verify_bootloader(self, expected_crc: int) -> None:
        """Send VERIFY command with expected CRC32"""
        output(f"Verifying bootloader CRC: 0x{expected_crc:08x}...")
        # Payload: CRC32 (4 bytes, little endian)
        payload = struct.pack("<I", expected_crc)
        await self._send_admin_command(ADMIN_CMD_BOOTLOADER_UPDATE_VERIFY, payload)
        
        try:
            resp_data = await self._wait_response(
                timeout=3.0, expected_cmd=ADMIN_CMD_BOOTLOADER_UPDATE_VERIFY)
            # Response: [CMD_ECHO] [CRC32 (4 bytes)] [verified (1 byte)]
            if len(resp_data) >= 6 and resp_data[5] == 0x01:
                device_crc = struct.unpack_from("<I", resp_data, 1)[0]
                output(f"Bootloader verification successful (device CRC: 0x{device_crc:08x})")
            elif len(resp_data) >= 6:
                device_crc = struct.unpack_from("<I", resp_data, 1)[0]
                raise RuntimeError(
                    f"Bootloader verification FAILED on device "
                    f"(expected CRC: 0x{expected_crc:08x}, device CRC: 0x{device_crc:08x})")
            else:
                raise RuntimeError(f"Unexpected verify response: {resp_data.hex()}")
        except TimeoutError:
            raise RuntimeError("VERIFY command timed out")
    
    async def commit_bootloader(self) -> None:
        """Send COMMIT command to flash bootloader to active area"""
        output("Committing bootloader to flash...")
        await self._send_admin_command(ADMIN_CMD_BOOTLOADER_UPDATE_COMMIT)
        
        try:
            resp_data = await self._wait_response(
                timeout=10.0,
                expected_cmd=ADMIN_CMD_BOOTLOADER_UPDATE_COMMIT)  # Longer timeout for flash operation
            # Response: [CMD_ECHO] [success (1 byte)]
            if len(resp_data) >= 2 and resp_data[1] == 0x01:
                output("Bootloader successfully flashed!")
            elif len(resp_data) >= 2:
                raise RuntimeError("COMMIT rejected by device (verification may have failed)")
            else:
                raise RuntimeError(f"Unexpected commit response: {resp_data.hex()}")
        except TimeoutError:
            raise RuntimeError("COMMIT command timed out")
    
    async def abort_update(self) -> None:
        """Send ABORT command"""
        output("Aborting bootloader update...")
        await self._send_admin_command(ADMIN_CMD_BOOTLOADER_UPDATE_ABORT)
    
    async def flash(self) -> None:
        """Main flashing routine"""
        # Read bootloader binary
        output(f"Reading bootloader from {self.bootloader_path}")
        bootloader_data = self.bootloader_path.read_bytes()
        
        if len(bootloader_data) > BOOTLOADER_MAX_SIZE:
            raise ValueError(f"Bootloader too large: {len(bootloader_data)} > {BOOTLOADER_MAX_SIZE}")
        
        # Calculate CRC32
        bl_crc = crc32(bootloader_data)
        
        # Open CAN socket
        output(f"Opening CAN interface: {self.can_interface}")
        self.cansock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.cansock.bind((self.can_interface,))
        except OSError as e:
            raise RuntimeError(f"Failed to bind to {self.can_interface}: {e}")
        
        self.cansock.setblocking(False)
        self.loop = asyncio.get_event_loop()
        self.loop.add_reader(self.cansock.fileno(), self._handle_can_response)
        
        try:
            # Start update
            await self.start_update(len(bootloader_data))
            
            # Send data in chunks
            total_chunks = (len(bootloader_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
            output(f"Sending {total_chunks} chunks ({len(bootloader_data)} bytes)...")
            
            next_page_boundary = FLASH_PAGE_SIZE
            for i in range(0, len(bootloader_data), CHUNK_SIZE):
                chunk = bootloader_data[i:i+CHUNK_SIZE]
                await self.send_chunk(i, chunk)
                
                # Progress indicator
                if i % 500 == 0:
                    progress = (i / len(bootloader_data)) * 100
                    output(f"Progress: {progress:.1f}% ({i}/{len(bootloader_data)} bytes)")
                
                # Flow control: wait for page-write ACK when a page
                # boundary is crossed. The firmware ACKs only after
                # completing a flash page write, so we must pause here
                # to avoid overflowing the STM32 CAN RX FIFO while the
                # CPU is stalled programming flash.
                chunk_end = min(i + CHUNK_SIZE, len(bootloader_data))
                if chunk_end >= next_page_boundary:
                    try:
                        await self._wait_response(
                            timeout=5.0,
                            expected_cmd=ADMIN_CMD_BOOTLOADER_UPDATE_CHUNK)
                    except TimeoutError:
                        raise RuntimeError(
                            f"Page write ACK timed out at byte {chunk_end}")
                    next_page_boundary += FLASH_PAGE_SIZE
            
            output(f"All data sent ({len(bootloader_data)} bytes)")
            
            # Small delay before verify
            await asyncio.sleep(0.5)
            
            # Verify
            await self.verify_bootloader(bl_crc)
            
            # Commit
            await self.commit_bootloader()
            
        except Exception as e:
            output(f"Error during flash: {e}")
            await self.abort_update()
            raise
        finally:
            if self.loop and self.cansock:
                self.loop.remove_reader(self.cansock.fileno())
            if self.cansock:
                self.cansock.close()


async def main():
    parser = argparse.ArgumentParser(description="Flash bootloader over CAN using admin channel")
    parser.add_argument("-i", "--interface", required=True, help="CAN interface (e.g., can0)")
    parser.add_argument("-u", "--uuid", required=True, help="Target device UUID (12 hex chars)")
    parser.add_argument("-f", "--file", required=True, type=pathlib.Path, help="Bootloader binary file")
    
    args = parser.parse_args()
    
    if not args.file.exists():
        output(f"Error: Bootloader file not found: {args.file}")
        return 1
    
    flasher = BootloaderFlasher(args.interface, args.uuid, args.file)
    
    try:
        await flasher.flash()
        output("\nBootloader update completed successfully!")
        return 0
    except Exception as e:
        output(f"\nBootloader update failed: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
