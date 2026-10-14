"""Run:  python -m unittest discover -s tests   (from the tools/ directory)"""
import math
import unittest

import numpy as np

from mpu6050_viewer.fusion import Madgwick, q_from_euler, q_to_euler, q_to_matrix
from mpu6050_viewer.pipeline import Pipeline
from mpu6050_viewer.protocol import FrameParser, SampleDecoder, crc16, encode_frame
from mpu6050_viewer.sources import DemoSource


class ProtocolTests(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)    # CRC-16/CCITT-FALSE reference

    def test_roundtrip_and_scaling(self):
        raw = (100, -200, 16384, 0, 131, -262, 0)
        frames = FrameParser().feed(encode_frame(0, 7, 123456, raw))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].raw, raw)
        s = SampleDecoder().decode(frames[0])
        np.testing.assert_allclose(s.accel, [100 / 16384, -200 / 16384, 1.0])
        np.testing.assert_allclose(s.gyro, [1.0, -2.0, 0.0])

    def test_resync_after_garbage_and_text(self):
        good = encode_frame(0, 1, 10, (1, 2, 3, 4, 5, 6, 7))
        p = FrameParser()
        stream = b"MPU6050 demo\r\n" + b"\xAA" + good[:10]       # text, stray 0xAA, half a frame
        out = p.feed(stream)
        out += p.feed(good[10:] + good)                           # rest + one more frame
        self.assertEqual([f.raw for f in out], [(1, 2, 3, 4, 5, 6, 7)] * 2)

    def test_bad_crc_is_dropped(self):
        good = bytearray(encode_frame(0, 1, 10, (1, 2, 3, 4, 5, 6, 7)))
        bad = bytearray(good)
        bad[9] ^= 0xFF
        p = FrameParser()
        out = p.feed(bytes(bad) + bytes(good))
        self.assertEqual(len(out), 1)
        self.assertEqual(p.crc_errors, 1)

    def test_timestamp_wrap_and_lost_frames(self):
        d = SampleDecoder()
        mk = lambda seq, t: FrameParser().feed(encode_frame(0, seq, t, (0,) * 7))[0]
        a = d.decode(mk(254, 0xFFFFFFFF - 4_999))
        b = d.decode(mk(0, 5_000))             # wrapped timestamp, seq 255 missing
        self.assertAlmostEqual(b.t - a.t, 0.01, places=6)
        self.assertEqual(d.lost_frames, 1)


class FusionTests(unittest.TestCase):
    def test_euler_roundtrip(self):
        for r, p, y in [(0.3, -0.5, 1.0), (-1.0, 0.2, -2.5)]:
            np.testing.assert_allclose(q_to_euler(q_from_euler(r, p, y)), (r, p, y), atol=1e-9)

    def test_flat_is_identity(self):
        m = Madgwick()
        m.update(np.zeros(3), np.array([0, 0, 1.0]), 0.01)
        np.testing.assert_allclose(m.euler_deg, 0, atol=1e-6)

    def test_static_tilt_from_gravity(self):
        roll, pitch = math.radians(30), math.radians(-20)
        g_body = q_to_matrix(q_from_euler(roll, pitch, 0)).T @ [0, 0, 1.0]
        m = Madgwick()
        for _ in range(100):
            m.update(np.zeros(3), g_body, 0.01)
        np.testing.assert_allclose(m.euler_deg[:2], [30, -20], atol=0.1)

    def test_tracks_simulated_motion(self):
        """Fuse the demo stream and compare tilt with the simulator's ground truth."""
        src, parser, dec = DemoSource(), FrameParser(), SampleDecoder()
        pipe = Pipeline(Madgwick(beta=0.1))
        src.open()
        errs = []
        for _ in range(3000):                  # 30 s of simulated time, no real waiting
            frame = src._next_frame()
            for f in parser.feed(frame):
                pipe.process(dec.decode(f))
            tr, tp, _ = q_to_euler(src.true_quaternion())
            er, ep, _ = np.radians(pipe.estimator.euler_deg)
            errs.append(max(abs(tr - er), abs(tp - ep)))
        self.assertLess(math.degrees(np.mean(errs[500:])), 3.0)
        self.assertEqual(parser.crc_errors, 0)
        self.assertEqual(dec.lost_frames, 0)

    def test_reset_yaw(self):
        m = Madgwick()
        m._q = q_from_euler(0.2, 0.1, math.radians(75))
        m._initialized = True
        m.reset_yaw()
        r, p, y = m.euler_deg
        self.assertAlmostEqual(y, 0, places=6)
        self.assertAlmostEqual(r, math.degrees(0.2), places=4)
        self.assertAlmostEqual(p, math.degrees(0.1), places=4)


class PipelineTests(unittest.TestCase):
    def _feed(self, pipe, n, gyro, t0=0.0):
        from mpu6050_viewer.models import Sample
        for i in range(n):
            pipe.process(Sample(t0 + i * 0.01, i & 0xFF, np.array([0, 0, 1.0]), np.array(gyro, float), 25.0))

    def test_gyro_calibration_removes_bias(self):
        pipe = Pipeline(Madgwick())
        self._feed(pipe, 5, [0.4, -0.2, 0.1])
        pipe.start_gyro_calibration(1.0)
        self._feed(pipe, 150, [0.4, -0.2, 0.1], t0=0.05)
        self.assertFalse(pipe.calibrating)
        np.testing.assert_allclose(pipe.gyro_bias, [0.4, -0.2, 0.1], atol=1e-9)

    def test_calibration_rejected_when_moving(self):
        from mpu6050_viewer.models import Sample
        pipe = Pipeline(Madgwick())
        pipe.start_gyro_calibration(1.0)
        rng = np.random.default_rng(0)
        for i in range(150):
            pipe.process(Sample(i * 0.01, i, np.array([0, 0, 1.0]), rng.normal(0, 20, 3), 25.0))
        np.testing.assert_allclose(pipe.gyro_bias, 0)
        self.assertIn("rejected", pipe.calibration_message)


if __name__ == "__main__":
    unittest.main()
