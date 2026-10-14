"""Three stacked live plots: accelerometer, gyroscope, Euler angles."""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QVBoxLayout, QWidget

AXIS_COLORS = ("#e5484d", "#30a46c", "#3e63dd")     # x, y, z


class PlotPanel(QWidget):
    WINDOW_S = 10.0

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        spec = [("accel", "Accelerometer [g]", ("ax", "ay", "az")),
                ("gyro", "Gyroscope [deg/s]", ("gx", "gy", "gz")),
                ("euler", "Orientation [deg]", ("roll", "pitch", "yaw"))]
        self._curves: dict[str, list[pg.PlotDataItem]] = {}
        first = None
        for key, title, names in spec:
            plot = pg.PlotWidget()
            plot.setTitle(title, size="9pt")
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend(offset=(5, 5), labelTextSize="8pt")
            plot.setXRange(-self.WINDOW_S, 0, padding=0)
            plot.setMouseEnabled(x=False, y=True)
            plot.enableAutoRange(axis="y")
            if first is None:
                first = plot
            else:
                plot.setXLink(first)
            self._curves[key] = [plot.plot(pen=pg.mkPen(c, width=1.5), name=n)
                                 for c, n in zip(AXIS_COLORS, names)]
            layout.addWidget(plot)
        first.getPlotItem().getAxis("bottom").setLabel("time [s] (0 = now)")

    def update_data(self, snap: dict[str, np.ndarray]) -> None:
        t = snap["t"]
        if len(t) == 0:
            return
        x = t - t[-1]
        for key, curves in self._curves.items():
            data = snap[key]
            for i, curve in enumerate(curves):
                curve.setData(x, data[:, i])

    def clear(self) -> None:
        for curves in self._curves.values():
            for curve in curves:
                curve.setData([], [])
