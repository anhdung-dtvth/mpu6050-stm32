"""Byte sources: real serial port or a built-in simulator that speaks the real protocol."""
from __future__ import annotations

import time
from abc import ABC, abstractmethod

import numpy as np

from .fusion import q_mul, q_to_matrix
from .protocol import encode_frame


class DataSource(ABC):
    """Anything that produces raw bytes of the frame stream."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def read(self) -> bytes:
        """Return whatever is available (may be empty). Must not block for long."""

    @abstractmethod
    def close(self) -> None: ...


class SerialSource(DataSource):
    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        self.baud = baud          # ignored by USB CDC, kept for plain UART bridges
        self._ser = None

    def open(self) -> None:
        import serial  # pyserial
        self._ser = serial.Serial(self.port, self.baud, timeout=0.02)

    def read(self) -> bytes:
        n = self._ser.in_waiting
        return self._ser.read(n if n else 1)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None


class DemoSource(DataSource):
    """Simulates a sensor being waved around, at 100 Hz, as real binary frames.

    Body rates are sinusoids that are integrated into a true orientation; the
    simulated accelerometer sees gravity in the body frame and the gyro has a
    small constant bias plus noise, like the real chip.
    """

    PERIOD = 0.01

    def __init__(self, seed: int = 1) -> None:
        self._rng = np.random.default_rng(seed)
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self._n = 0
        self._start = 0.0

    def open(self) -> None:
        self._start = time.perf_counter()
        self._n = 0

    def read(self) -> bytes:
        due = int((time.perf_counter() - self._start) / self.PERIOD) - self._n
        if due <= 0:
            time.sleep(0.002)
            return b""
        return b"".join(self._next_frame() for _ in range(due))

    def close(self) -> None:
        pass

    def true_quaternion(self) -> np.ndarray:
        return self._q.copy()

    def _next_frame(self) -> bytes:
        t = self._n * self.PERIOD
        omega = np.array([40 * np.sin(0.8 * t), 30 * np.cos(0.5 * t), 20 * np.sin(0.3 * t)])  # deg/s, body frame
        self._q = self._q + 0.5 * q_mul(self._q, np.array([0.0, *np.radians(omega)])) * self.PERIOD
        self._q /= np.linalg.norm(self._q)

        accel = q_to_matrix(self._q).T @ np.array([0.0, 0.0, 1.0]) + self._rng.normal(0, 0.004, 3)
        gyro = omega + np.array([0.5, -0.3, 0.2]) + self._rng.normal(0, 0.05, 3)
        raw = (*np.round(accel * 16384.0).astype(int),
               int(round((26.5 - 36.53) * 340.0)),
               *np.round(gyro * 131.0).astype(int))
        frame = encode_frame(0, self._n, int(round(t * 1e6)), tuple(int(v) for v in raw))
        self._n += 1
        return frame
