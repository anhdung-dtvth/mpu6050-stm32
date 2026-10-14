"""Main window + controller: wires source -> thread -> pipeline -> views."""
from __future__ import annotations

import argparse
import sys
from datetime import datetime

import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (QApplication, QFileDialog, QMainWindow, QMessageBox, QSplitter)

from .fusion import Madgwick
from .pipeline import Pipeline
from .recorder import CsvRecorder
from .sources import DataSource, DemoSource, SerialSource
from .views.controls import DEMO_KEY, ControlBar
from .views.plots import PlotPanel
from .views.scene3d import Orientation3DView
from .worker import ReaderThread

REFRESH_MS = 33          # ~30 fps GUI refresh; data processing runs at the sensor rate


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("MPU6050 viewer")
        self.resize(1300, 800)

        self.estimator = Madgwick(beta=0.1)
        self.pipeline = Pipeline(self.estimator)
        self.thread: ReaderThread | None = None

        self.controls = ControlBar(beta=self.estimator.beta)
        self.plots = PlotPanel()
        self.scene = Orientation3DView()

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.plots)
        split.addWidget(self.scene)
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 4)

        central = QSplitter(Qt.Vertical)
        central.addWidget(self.controls)
        central.addWidget(split)
        central.setCollapsible(0, False)
        central.setSizes([40, 760])
        central.setHandleWidth(1)
        self.setCentralWidget(central)

        c = self.controls
        c.connect_requested.connect(self.connect_source)
        c.disconnect_requested.connect(self.disconnect_source)
        c.record_toggled.connect(self.toggle_record)
        c.calibrate_requested.connect(lambda: self.pipeline.start_gyro_calibration())
        c.reset_yaw_requested.connect(lambda: self.pipeline.reset_yaw())
        c.reset_filter_requested.connect(lambda: self.pipeline.reset_filter())
        c.beta_changed.connect(self._set_beta)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(REFRESH_MS)
        self.statusBar().showMessage("disconnected")

    # --- connection ---------------------------------------------------------
    def connect_source(self, key: str) -> None:
        source: DataSource = DemoSource() if key == DEMO_KEY else SerialSource(key)
        self.pipeline = self._fresh_pipeline()
        self.plots.clear()
        self.thread = ReaderThread(source)
        self.thread.batch.connect(self._on_batch)
        self.thread.failed.connect(self._on_failed)
        self.thread.start()
        self.controls.set_connected(True)

    def disconnect_source(self) -> None:
        if self.thread is not None:
            self.thread.stop()
            self.thread = None
        self._stop_recording()
        self.controls.set_connected(False)
        self.statusBar().showMessage("disconnected")

    def _fresh_pipeline(self) -> Pipeline:
        self.estimator = Madgwick(beta=self.controls.beta_spin.value())
        return Pipeline(self.estimator)

    def _on_batch(self, samples: list) -> None:
        for s in samples:
            self.pipeline.process(s)

    def _on_failed(self, message: str) -> None:
        self.disconnect_source()
        QMessageBox.warning(self, "Connection problem", message)

    # --- actions ------------------------------------------------------------
    def _set_beta(self, value: float) -> None:
        self.estimator.beta = value

    def toggle_record(self, on: bool) -> None:
        if not on:
            self._stop_recording()
            return
        default = datetime.now().strftime("mpu6050_%Y%m%d_%H%M%S.csv")
        path, _ = QFileDialog.getSaveFileName(self, "Record to CSV", default, "CSV (*.csv)")
        if not path:
            self.controls.record_btn.blockSignals(True)
            self.controls.record_btn.setChecked(False)
            self.controls.record_btn.blockSignals(False)
            return
        self.pipeline.recorder = CsvRecorder(path)

    def _stop_recording(self) -> None:
        rec, self.pipeline.recorder = self.pipeline.recorder, None
        if rec is not None:
            rec.close()
            self.statusBar().showMessage(f"saved {rec.rows} rows to {rec.path}", 5000)

    # --- periodic refresh ---------------------------------------------------
    def _refresh(self) -> None:
        p = self.pipeline
        self.plots.update_data(p.snapshot())
        if p.samples_processed:
            self.scene.set_orientation(self.estimator.quaternion, self.estimator.euler_deg)
        if self.thread is not None:
            msg = (f"{p.rate_hz:5.1f} Hz | frames {self.thread.parser.frames_ok} | "
                   f"crc errors {self.thread.parser.crc_errors} | lost {self.thread.decoder.lost_frames}")
            if p.recorder is not None:
                msg += f" | REC {p.recorder.rows} rows"
            if p.calibration_message:
                msg += f" | {p.calibration_message}"
            self.statusBar().showMessage(msg)

    def closeEvent(self, event) -> None:
        self.disconnect_source()
        super().closeEvent(event)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="MPU6050 live viewer")
    ap.add_argument("--demo", action="store_true", help="start with the simulated sensor")
    args = ap.parse_args(argv)

    pg.setConfigOptions(antialias=True, background="#16181c", foreground="#c9ccd1")
    app = QApplication(sys.argv[:1])
    win = MainWindow()
    win.show()
    if args.demo:
        win.controls.select_demo()
        win.controls.connect_btn.setChecked(True)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
