"""CSV recording of processed samples."""
from __future__ import annotations

import csv

import numpy as np

from .models import Sample


class CsvRecorder:
    HEADER = ["t_s", "seq", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "temp_c",
              "qw", "qx", "qy", "qz", "roll_deg", "pitch_deg", "yaw_deg"]

    def __init__(self, path: str) -> None:
        self.path = path
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self.rows = 0

    def write(self, s: Sample, gyro_corrected: np.ndarray, quat: np.ndarray, euler_deg: np.ndarray) -> None:
        self._writer.writerow([f"{s.t:.6f}", s.seq,
                               *(f"{v:.5f}" for v in s.accel),
                               *(f"{v:.4f}" for v in gyro_corrected),
                               f"{s.temp_c:.2f}",
                               *(f"{v:.6f}" for v in quat),
                               *(f"{v:.3f}" for v in euler_deg)])
        self.rows += 1

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()
