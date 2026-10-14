"""Top control bar. Emits signals only; knows nothing about sources or processing."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QWidget)

DEMO_KEY = "demo"


class ControlBar(QWidget):
    connect_requested = Signal(str)       # port name, or DEMO_KEY
    disconnect_requested = Signal()
    record_toggled = Signal(bool)
    calibrate_requested = Signal()
    reset_yaw_requested = Signal()
    reset_filter_requested = Signal()
    beta_changed = Signal(float)

    def __init__(self, beta: float = 0.1, parent=None) -> None:
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(6, 4, 6, 4)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(260)
        self.refresh_btn = QPushButton("Refresh")
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setCheckable(True)
        self.record_btn = QPushButton("Record")
        self.record_btn.setCheckable(True)
        self.record_btn.setEnabled(False)
        self.calibrate_btn = QPushButton("Calibrate gyro")
        self.yaw_btn = QPushButton("Reset yaw")
        self.filter_btn = QPushButton("Reset filter")
        self.beta_spin = QDoubleSpinBox()
        self.beta_spin.setRange(0.005, 2.0)
        self.beta_spin.setSingleStep(0.01)
        self.beta_spin.setDecimals(3)
        self.beta_spin.setValue(beta)
        self.beta_spin.setToolTip("Madgwick gain: higher trusts the accelerometer more")

        for w in (QLabel("Port"), self.port_combo, self.refresh_btn, self.connect_btn):
            row.addWidget(w)
        row.addSpacing(16)
        for w in (self.record_btn, self.calibrate_btn, self.yaw_btn, self.filter_btn):
            row.addWidget(w)
        row.addSpacing(16)
        row.addWidget(QLabel("Madgwick β"))
        row.addWidget(self.beta_spin)
        row.addStretch(1)
        self.set_actions_enabled(False)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.toggled.connect(self._on_connect_toggled)
        self.record_btn.toggled.connect(self.record_toggled)
        self.calibrate_btn.clicked.connect(self.calibrate_requested)
        self.yaw_btn.clicked.connect(self.reset_yaw_requested)
        self.filter_btn.clicked.connect(self.reset_filter_requested)
        self.beta_spin.valueChanged.connect(self.beta_changed)
        self.refresh_ports()

    def refresh_ports(self) -> None:
        from serial.tools import list_ports
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        self.port_combo.addItem("Demo (simulated sensor)", DEMO_KEY)
        idx = self.port_combo.findData(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def select_demo(self) -> None:
        self.port_combo.setCurrentIndex(self.port_combo.findData(DEMO_KEY))

    def set_actions_enabled(self, on: bool) -> None:
        for w in (self.record_btn, self.calibrate_btn, self.yaw_btn, self.filter_btn):
            w.setEnabled(on)

    def set_connected(self, connected: bool) -> None:
        """Sync the buttons with the real state (e.g. after a read error)."""
        self.connect_btn.blockSignals(True)
        self.connect_btn.setChecked(connected)
        self.connect_btn.blockSignals(False)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.port_combo.setEnabled(not connected)
        self.refresh_btn.setEnabled(not connected)
        self.set_actions_enabled(connected)
        if not connected:
            self.record_btn.blockSignals(True)
            self.record_btn.setChecked(False)
            self.record_btn.blockSignals(False)

    def _on_connect_toggled(self, checked: bool) -> None:
        if checked:
            self.connect_requested.emit(self.port_combo.currentData())
        else:
            self.disconnect_requested.emit()
