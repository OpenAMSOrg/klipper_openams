# OpenAMS Moonraker helpers
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional, Literal
from urllib.parse import urlparse, urlunparse
from urllib.request import Request, urlopen


class OpenAMSMoonrakerClient:
    """Minimal Moonraker client for OpenAMS-owned status publishing.

    This client is intentionally scoped to OpenAMS integration so we avoid
    modifying AFC core Moonraker behavior.
    """

    def __init__(self, host: str, port: int, logger) -> None:
        self.base_url = self._build_base_url(host, port)
        self.database_url = self.base_url + "/server/database/item"
        self.logger = logger
        self._last_status_fingerprint: Optional[str] = None

    @staticmethod
    def _build_base_url(host: str, port: int) -> str:
        """Build base URL, handling cases where *host* already contains a port."""
        host = host.strip().rstrip("/")
        parsed = urlparse(host)
        # If host was just "localhost" without scheme, urlparse puts it in path
        if not parsed.scheme:
            parsed = urlparse(f"http://{host}")
        # Always use the explicitly provided port, ignoring any port in the host
        netloc = parsed.hostname or "localhost"
        return urlunparse((parsed.scheme or "http", f"{netloc}:{int(port)}", "", "", "", ""))

    def _request(self, url: str, *, method: str = "GET",
                 data: Optional[bytes] = None,
                 timeout: float = 1.5) -> Optional[Dict[str, Any]]:
        headers = {"Content-Type": "application/json"} if data else {}
        req = Request(url, data=data, method=method, headers=headers)
        try:
            with urlopen(req, timeout=timeout) as response:
                payload = json.load(response)
                return payload.get("result") if isinstance(payload, dict) else None
        except Exception as exc:
            self.logger.debug(f"OpenAMS Moonraker request error: {exc}")
            return None

    def _request_with_retry(self, url: str, *, method: str = "GET",
                            data: Optional[bytes] = None,
                            timeout: float = 1.5) -> Optional[Dict[str, Any]]:
        """Issue a request, retrying once after a brief pause on failure."""
        result = self._request(url, method=method, data=data, timeout=timeout)
        if result is not None:
            return result
        time.sleep(0.25)
        return self._request(url, method=method, data=data, timeout=timeout)

    def is_available(self) -> bool:
        return self._request(self.base_url + "/server/info", timeout=1.0) is not None

    @staticmethod
    def _status_fingerprint(status: Dict[str, Any]) -> str:
        return json.dumps(status, sort_keys=True, separators=(",", ":"))

    # -- write -----------------------------------------------------------

    def publish_manager_status(
        self,
        status: Dict[str, Any],
        *,
        eventtime: float,
    ) -> Literal["published", "skipped", "failed"]:
        fingerprint = self._status_fingerprint(status)
        if fingerprint == self._last_status_fingerprint:
            return "skipped"

        payload = json.dumps({
            "request_method": "POST",
            "namespace": "openams",
            "key": "manager_status",
            "value": {
                "eventtime": eventtime,
                "status": status,
            },
        }).encode()

        result = self._request_with_retry(
            self.database_url, method="POST", data=payload,
        )
        if result is None:
            return "failed"

        self._last_status_fingerprint = fingerprint
        return "published"

    # -- read ------------------------------------------------------------

    def read_manager_status(self) -> Optional[Dict[str, Any]]:
        """Read the last-published manager status from the Moonraker database.

        Returns the stored ``{"eventtime": ..., "status": ...}`` dict, or
        *None* if the key doesn't exist or the request fails.
        """
        url = self.database_url + "?namespace=openams&key=manager_status"
        result = self._request(url, timeout=2.0)
        if not isinstance(result, dict):
            return None
        value = result.get("value")
        return value if isinstance(value, dict) else None
