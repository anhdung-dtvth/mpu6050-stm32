"""Orientation estimation.

Conventions
-----------
* Quaternion q = [w, x, y, z] rotates *sensor-frame* vectors into the *earth* frame
  (earth: Z up). Lying flat on a table with Z up gives the identity quaternion.
* Euler angles are ZYX (yaw-pitch-roll), returned in degrees.
* A 6-axis IMU has no heading reference, so yaw can only come from integrating
  the gyro and WILL drift (see OrientationEstimator.reset_yaw).
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod

import numpy as np


# --- quaternion helpers -----------------------------------------------------

def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def q_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler angles in radians -> quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def q_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """Quaternion -> (roll, pitch, yaw) in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def q_to_matrix(q: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix (sensor -> earth)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# --- estimators -------------------------------------------------------------

class OrientationEstimator(ABC):
    """Base class: subclasses implement update()."""

    def __init__(self) -> None:
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self._initialized = False

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    @property
    def euler_deg(self) -> np.ndarray:
        return np.degrees(q_to_euler(self._q))

    def reset(self) -> None:
        """Forget the orientation; the next update re-initialises from gravity."""
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        self._initialized = False

    def reset_yaw(self) -> None:
        """Make the current heading the zero reference (rotation about earth Z)."""
        yaw = q_to_euler(self._q)[2]
        self._q = q_mul(q_from_euler(0.0, 0.0, -yaw), self._q)
        self._q /= np.linalg.norm(self._q)

    def _init_from_accel(self, accel: np.ndarray) -> bool:
        if np.linalg.norm(accel) < 1e-6:
            return False
        ax, ay, az = accel
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.hypot(ay, az))
        self._q = q_from_euler(roll, pitch, 0.0)
        self._initialized = True
        return True

    @abstractmethod
    def update(self, gyro_dps: np.ndarray, accel_g: np.ndarray, dt: float) -> np.ndarray:
        """Feed one sample, return the new quaternion."""


class Madgwick(OrientationEstimator):
    """Madgwick gradient-descent filter, IMU (gyro + accel) variant.

    beta is the filter gain: larger = trusts the accelerometer more (faster
    convergence, more sensitivity to linear acceleration/vibration).
    """

    def __init__(self, beta: float = 0.1) -> None:
        super().__init__()
        self.beta = beta

    def update(self, gyro_dps: np.ndarray, accel_g: np.ndarray, dt: float) -> np.ndarray:
        if not self._initialized and not self._init_from_accel(accel_g):
            return self.quaternion
        if dt <= 0.0:
            return self.quaternion

        gyro = np.radians(gyro_dps)
        q = self._q
        q0, q1, q2, q3 = q
        q_dot = 0.5 * q_mul(q, np.array([0.0, *gyro]))

        norm = float(np.linalg.norm(accel_g))
        if norm > 1e-6:
            ax, ay, az = accel_g / norm
            # objective: predicted gravity direction in the sensor frame minus measured
            f = np.array([
                2 * (q1 * q3 - q0 * q2) - ax,
                2 * (q0 * q1 + q2 * q3) - ay,
                2 * (0.5 - q1 * q1 - q2 * q2) - az,
            ])
            jac = np.array([
                [-2 * q2, 2 * q3, -2 * q0, 2 * q1],
                [2 * q1, 2 * q0, 2 * q3, 2 * q2],
                [0.0, -4 * q1, -4 * q2, 0.0],
            ])
            step = jac.T @ f
            n = float(np.linalg.norm(step))
            if n > 0.0:
                q_dot -= self.beta * step / n

        self._q = q + q_dot * dt
        self._q /= np.linalg.norm(self._q)
        return self.quaternion
