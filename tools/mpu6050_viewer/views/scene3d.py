"""3D view: a board-shaped box that follows the estimated orientation."""
from __future__ import annotations

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph import Transform3D
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from ..fusion import q_to_matrix

# Half sizes of the box in sensor axes (x forward, y left, z up), arbitrary units
HALF = np.array([1.0, 0.7, 0.12])

# face -> (outward axis index, sign, RGBA)
_FACES = [
    (2, +1, (0.24, 0.39, 0.87, 1.0)),   # +Z top    blue
    (2, -1, (0.35, 0.35, 0.38, 1.0)),   # -Z bottom grey
    (0, +1, (0.90, 0.28, 0.30, 1.0)),   # +X front  red
    (0, -1, (0.50, 0.15, 0.17, 1.0)),   # -X back   dark red
    (1, +1, (0.19, 0.64, 0.42, 1.0)),   # +Y left   green
    (1, -1, (0.10, 0.36, 0.24, 1.0)),   # -Y right  dark green
]


def _box_mesh() -> gl.MeshData:
    verts, faces, colors = [], [], []
    for axis, sign, rgba in _FACES:
        u, v = [i for i in range(3) if i != axis]
        base = len(verts)
        for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):     # one quad per face
            p = np.zeros(3)
            p[axis], p[u], p[v] = sign * HALF[axis], su * HALF[u], sv * HALF[v]
            verts.append(p)
        faces += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
        colors += [rgba, rgba]
    return gl.MeshData(vertexes=np.array(verts), faces=np.array(faces), faceColors=np.array(colors))


class Orientation3DView(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._readout = QLabel("roll  --   pitch  --   yaw  --")
        self._readout.setStyleSheet("font: 600 13pt 'Consolas'; padding: 4px;")
        layout.addWidget(self._readout)

        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor((22, 24, 28))
        self._view.setCameraPosition(distance=7, elevation=25, azimuth=-60)
        layout.addWidget(self._view, 1)

        grid = gl.GLGridItem()
        grid.setSize(8, 8)
        grid.setSpacing(1, 1)
        grid.setColor((255, 255, 255, 40))
        self._view.addItem(grid)

        # fixed world axes (earth frame): thin lines at the corner of the grid
        for vec, rgba in (((2, 0, 0), (1, .3, .3, .5)), ((0, 2, 0), (.3, 1, .4, .5)), ((0, 0, 2), (.4, .6, 1, .5))):
            line = gl.GLLinePlotItem(pos=np.array([(0, 0, 0), vec], dtype=float), color=rgba, width=1, antialias=True)
            self._view.addItem(line)

        # body: box + body axes (both rotated by the same matrix)
        self._body = [gl.GLMeshItem(meshdata=_box_mesh(), smooth=False, shader="shaded", drawEdges=True,
                                    edgeColor=(1, 1, 1, 0.35))]
        for vec, rgba in (((1.8, 0, 0), (1, .3, .3, 1)), ((0, 1.4, 0), (.3, 1, .4, 1)), ((0, 0, 1.2), (.4, .6, 1, 1))):
            self._body.append(gl.GLLinePlotItem(pos=np.array([(0, 0, 0), vec], dtype=float), color=rgba,
                                                width=3, antialias=True))
        for item in self._body:
            self._view.addItem(item)
        self.set_orientation(np.array([1.0, 0, 0, 0]), (0.0, 0.0, 0.0))

    def set_orientation(self, q: np.ndarray, euler_deg) -> None:
        m = np.eye(4)
        m[:3, :3] = q_to_matrix(q)
        tf = Transform3D(m.tolist())
        for item in self._body:
            item.setTransform(tf)
        roll, pitch, yaw = euler_deg
        self._readout.setText(f"roll {roll:+7.1f}°   pitch {pitch:+7.1f}°   yaw {yaw:+7.1f}°")
