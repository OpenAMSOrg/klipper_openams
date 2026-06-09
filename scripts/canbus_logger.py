# Tool to log CAN bus logging protocol from OAMS
#
# Copyright (C) 2024-2026  JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import sys, os, optparse, time
import can
from termcolor import colored, cprint
import argparse

LOGGING_BASE_ADDRESS = 0x780

log_levels = {0 : 'FATAL', 1 : 'ERROR', 2 : 'WARNING', 3 : 'INFO', 4 : 'DEBUG'}
log_level_colors = {0 : 'red', 1 : 'yellow', 2 : 'green', 3 : 'blue', 4 : 'dark_grey'}

def log(canbus_iface):
    # Open CAN socket
    filters = [{"can_id": LOGGING_BASE_ADDRESS, "can_mask": LOGGING_BASE_ADDRESS,
                "extended": False}]
    bus = can.interface.Bus(channel=canbus_iface, can_filters=filters,
                            bustype='socketcan')
    last_id = None
    while True:
        msg = bus.recv()
        # decode id
        id = msg.arbitration_id
        ams = id >> 3 & 0xF
        log_level = id & 0x7
        if last_id != id:
            color = 'white'
            if log_level == 0:
                color = 'red'
            elif log_level == 1:
                color = 'yellow'
            elif log_level == 2:
                color = 'green'
            elif log_level == 3:
                color = 'blue'
            elif log_level == 4:
                color = 'gray'
            #print("AMS: %d, Level: %s --> " % (ams log_levels[log_level]), color), end='')
            print(colored("AMS: ","cyan") + colored(ams, 'white') + ", " +
                  #colored("Serial: ","cyan") + colored(serial, 'white') + ", " +
                  colored("Level: ","cyan") + colored(log_levels[log_level], log_level_colors[log_level]) + " ", end = '')
        color = 'white'
        print(colored("%s" % msg.data.decode("utf8"), color), end='')
        last_id = id
        if chr(msg.data[-1]) == '\n':
            last_id = None
        
if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CAN bus logger')
    argparser.add_argument('canbus_iface', help='CAN bus interface')
    args = argparser.parse_args()
    canbus_iface = args.canbus_iface
    log(canbus_iface)
