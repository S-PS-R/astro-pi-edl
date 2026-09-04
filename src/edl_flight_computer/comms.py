"""Touchdown downlink over Pi 5 Wi-Fi (UDP). Bluetooth if EDL_BT_ADDR is set."""

from __future__ import annotations

import os
import socket

DEFAULT_HOST = os.environ.get("EDL_COMMS_HOST", "255.255.255.255")
DEFAULT_PORT = int(os.environ.get("EDL_COMMS_PORT", "5770"))
DEFAULT_BT_ADDR = os.environ.get("EDL_BT_ADDR", "")


def send_landing_message(text: str = "TOUCHDOWN", via: str = "wifi") -> None:
    if via not in {"wifi", "bluetooth"}:
        raise ValueError("via must be 'wifi' or 'bluetooth'")
    payload = text.encode("utf-8")
    if via == "wifi":
        _send_udp(payload)
        return
    _send_bluetooth(payload)


def _send_udp(payload: bytes) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(2.0)
        sock.sendto(payload, (DEFAULT_HOST, DEFAULT_PORT))
        print(f"[downlink] {payload!r} -> {DEFAULT_HOST}:{DEFAULT_PORT}")
    finally:
        sock.close()


def _send_bluetooth(payload: bytes) -> None:
    if not DEFAULT_BT_ADDR:
        raise RuntimeError("set EDL_BT_ADDR to the receiver MAC for Bluetooth")
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    try:
        sock.settimeout(5.0)
        sock.connect((DEFAULT_BT_ADDR, 1))
        sock.send(payload)
        print(f"[downlink] bluetooth {payload!r} -> {DEFAULT_BT_ADDR}")
    finally:
        sock.close()
