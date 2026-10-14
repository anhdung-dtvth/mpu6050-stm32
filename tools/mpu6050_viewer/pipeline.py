"""Processing chain: gyro bias -> orientation estimator -> history buffers -> recorder.

No Qt in here, so it can be unit-tested and driven from any source.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from .fusion import OrientationEstimator
from .models import Sample
from .recorder import CsvRecorder

NOMINAL_DT = 0.01
MAX_DT = 0.2                  # larger gaps are treated as a stall, not as real elapsed time
CAL_MAX_STD_DPS = 1.0         # gyro noise above this during calibration means the sensor moved


class Pipeline:
    def __init__(self, estimator: OrientationEstimator, history: int = 1000) -> None:
        self.estimator = estimator
        self.gyro_bias = np.zeros(3)
        self.recorder: CsvRecorder | None = None
        self.calibration_message = ""
        self.samples_processed = 0

        self._last_t: float | None = None
        self._t = deque(maxlen=history)
        self._accel = deque(maxlen=history)
        self._gyro = deque(maxlen=history)
        self._euler = deque(maxlen=history)

        self._cal_until: float | None = None
        self._cal_gyro: list[np.ndarray] = []

    # --- commands -----------------------------------------------------------
    def start_gyro_calibration(self, seconds: float = 2.0) -> None:
        """Average the gyro for `seconds` (sensor must be still) and use it as bias."""
        self._cal_until = (self._last_t or 0.0) + seconds
        self._cal_gyro = []
        self.calibration_message = "calibrating gyro... keep the sensor still"

    @property
    def calibrating(self) -> bool:
        return self._cal_until is not None

    def reset_filter(self) -> None:
        self.estimator.reset()

    def reset_yaw(self) -> None:
        self.estimator.reset_yaw()

    # --- data path ----------------------------------------------------------
    def process(self, s: Sample) -> None:
        if self._cal_until is not None:
            self._cal_gyro.append(s.gyro)
            if s.t >= self._cal_until:
                self._finish_calibration()

        dt = 0.0 if self._last_t is None else s.t - self._last_t
        if self._last_t is not None and not (0.0 < dt <= MAX_DT):
            dt = NOMINAL_DT
        self._last_t = s.t

        gyro = s.gyro - self.gyro_bias
        self.estimator.update(gyro, s.accel, dt)
        euler = self.estimator.euler_deg

        self._t.append(s.t)
        self._accel.append(s.accel)
        self._gyro.append(gyro)
        self._euler.append(euler)
        self.samples_processed += 1
        if self.recorder is not None:
            self.recorder.write(s, gyro, self.estimator.quaternion, euler)

    def _finish_calibration(self) -> None:
        data = np.array(self._cal_gyro)
        self._cal_until = None
        self._cal_gyro = []
        if len(data) < 20:
            self.calibration_message = "calibration failed: too few samples"
        elif np.max(data.std(axis=0)) > CAL_MAX_STD_DPS:
            self.calibration_message = "calibration rejected: sensor was moving, try again"
        else:
            self.gyro_bias = self.gyro_bias + data.mean(axis=0)
            self.calibration_message = "gyro bias = [%.3f %.3f %.3f] deg/s" % tuple(self.gyro_bias)

    # --- read side ----------------------------------------------------------
    @property
    def rate_hz(self) -> float:
        n = min(len(self._t), 100)
        if n < 2:
            return 0.0
        t = list(self._t)[-n:]
        return (n - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0

    def snapshot(self) -> dict[str, np.ndarray]:
        """Copy of the history for plotting."""
        if not self._t:
            empty = np.zeros((0, 3))
            return {"t": np.zeros(0), "accel": empty, "gyro": empty, "euler": empty}
        return {"t": np.array(self._t), "accel": np.array(self._accel),
                "gyro": np.array(self._gyro), "euler": np.array(self._euler)}
