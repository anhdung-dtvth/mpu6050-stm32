"""Plain data types shared by the whole tool (no Qt, no serial)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Index = range code sent in Frame.cfg (see App/Inc/frame.h in the firmware)
ACCEL_LSB_PER_G = (16384.0, 8192.0, 4096.0, 2048.0)
GYRO_LSB_PER_DPS = (131.0, 65.5, 32.8, 16.4)


@dataclass(frozen=True)
class Frame:
    """One decoded wire frame, values still raw."""
    cfg: int
    seq: int
    t_us: int
    raw: tuple[int, ...]    # ax, ay, az, temp, gx, gy, gz


@dataclass(frozen=True)
class Sample:
    """A frame converted to physical units."""
    t: float                # seconds since the first frame (timestamp wrap handled)
    seq: int
    accel: np.ndarray       # g,    shape (3,)
    gyro: np.ndarray        # deg/s, shape (3,)
    temp_c: float
