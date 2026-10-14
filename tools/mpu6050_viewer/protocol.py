"""Wire protocol: byte stream <-> Frame <-> Sample.

Layout (little endian, 24 bytes) - keep in sync with App/Inc/frame.h:
    0  2  sync 0xAA 0x55
    2  1  cfg   bits0-1 accel range, bits2-3 gyro range
    3  1  seq   +1 per frame, wraps at 256
    4  4  t_us  microsecond timestamp, wraps every ~71 min
    8 14  ax ay az temp gx gy gz (int16)
   22  2  CRC-16/CCITT-FALSE over bytes 2..21
"""
from __future__ import annotations

import struct

import numpy as np

from .models import ACCEL_LSB_PER_G, GYRO_LSB_PER_DPS, Frame, Sample

SYNC = b"\xAA\x55"
FRAME_LEN = 24
_STRUCT = struct.Struct("<2sBBI7hH")
assert _STRUCT.size == FRAME_LEN


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(cfg: int, seq: int, t_us: int, raw: tuple[int, ...]) -> bytes:
    """Build a frame exactly like the firmware does (used by the demo source and tests)."""
    body = struct.pack("<BBI7h", cfg, seq & 0xFF, t_us & 0xFFFFFFFF, *raw)
    return SYNC + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Turns an arbitrary byte stream into Frames.

    Resynchronises on the sync bytes after garbage or corruption, so ASCII
    text printed by the firmware (which never contains 0xAA 0x55) is skipped.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buf += data
        frames: list[Frame] = []
        buf = self._buf
        while True:
            i = buf.find(SYNC)
            if i < 0:
                keep = 1 if buf.endswith(SYNC[:1]) else 0     # a lone 0xAA may start a frame
                self.skipped_bytes += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if i > 0:
                self.skipped_bytes += i
                del buf[:i]
            if len(buf) < FRAME_LEN:
                break
            _, cfg, seq, t_us, *rest = _STRUCT.unpack_from(buf)
            raw, crc = tuple(rest[:7]), rest[7]
            if crc16(bytes(buf[2:22])) != crc:
                self.crc_errors += 1
                del buf[:2]          # false sync: look for the next one
                continue
            frames.append(Frame(cfg, seq, t_us, raw))
            self.frames_ok += 1
            del buf[:FRAME_LEN]
        return frames


class SampleDecoder:
    """Frame -> Sample: scales to g and deg/s, unwraps the timestamp, counts lost frames."""

    def __init__(self) -> None:
        self.lost_frames = 0
        self._t = 0.0
        self._last_us: int | None = None
        self._last_seq: int | None = None

    def decode(self, f: Frame) -> Sample:
        if self._last_us is not None:
            self._t += ((f.t_us - self._last_us) & 0xFFFFFFFF) * 1e-6
        if self._last_seq is not None:
            self.lost_frames += (f.seq - self._last_seq - 1) & 0xFF
        self._last_us, self._last_seq = f.t_us, f.seq

        a_lsb = ACCEL_LSB_PER_G[f.cfg & 0x3]
        g_lsb = GYRO_LSB_PER_DPS[(f.cfg >> 2) & 0x3]
        ax, ay, az, temp, gx, gy, gz = f.raw
        return Sample(
            t=self._t,
            seq=f.seq,
            accel=np.array([ax, ay, az], dtype=float) / a_lsb,
            gyro=np.array([gx, gy, gz], dtype=float) / g_lsb,
            temp_c=temp / 340.0 + 36.53,
        )
