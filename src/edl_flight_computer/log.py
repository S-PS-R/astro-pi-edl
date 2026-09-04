"""Telemetry writer: CSV on disk, optional UDP."""

from __future__ import annotations

import csv
import os
import socket
from pathlib import Path
from typing import Any, TextIO

from .comms import DEFAULT_HOST, DEFAULT_PORT

HEADER = (
    "t_s",
    "phase",
    "T_C",
    "ax",
    "ay",
    "az",
    "a_mag_g",
    "P_mbar",
    "fault",
    "touchdown",
)

DEFAULT_LOG_PATH = os.environ.get("EDL_LOG_PATH", "logs/edl_log.csv")


class TelemetryLog:
    def __init__(
        self,
        path: str | os.PathLike[str] = DEFAULT_LOG_PATH,
        udp: bool = True,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.host = host
        self.port = port
        new_file = not self.path.exists() or self.path.stat().st_size == 0
        self._fh: TextIO = self.path.open("a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        if new_file:
            self._writer.writerow(HEADER)
            self._fh.flush()
        self._sock: socket.socket | None = None
        if udp:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._sock.settimeout(0.05)

    def write(
        self,
        t_s: float,
        phase: str,
        temperature_c: float,
        accel: tuple[float, float, float],
        pressure_mbar: float,
        mag_g: float,
        fault: str = "none",
        touchdown: bool = False,
    ) -> str:
        ax, ay, az = accel
        row: list[Any] = [
            f"{t_s:.4f}",
            phase,
            f"{temperature_c:.3f}",
            f"{ax:.4f}",
            f"{ay:.4f}",
            f"{az:.4f}",
            f"{mag_g:.4f}",
            f"{pressure_mbar:.2f}",
            fault,
            int(touchdown),
        ]
        self._writer.writerow(row)
        self._fh.flush()
        line = ",".join(str(c) for c in row)
        if self._sock is not None:
            try:
                self._sock.sendto(line.encode("utf-8"), (self.host, self.port))
            except OSError:
                pass
        return line

    def close(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass

    def __enter__(self) -> "TelemetryLog":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


Recorder = TelemetryLog
