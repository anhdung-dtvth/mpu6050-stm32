"""Background reader: source bytes -> frames -> samples, delivered to the GUI in batches."""
from __future__ import annotations

import time

from PySide6.QtCore import QThread, Signal

from .protocol import FrameParser, SampleDecoder
from .sources import DataSource

BATCH_INTERVAL_S = 0.02


class ReaderThread(QThread):
    batch = Signal(list)      # list[Sample]
    failed = Signal(str)

    def __init__(self, source: DataSource, parent=None) -> None:
        super().__init__(parent)
        self.source = source
        self.parser = FrameParser()
        self.decoder = SampleDecoder()

    def run(self) -> None:
        try:
            self.source.open()
        except Exception as exc:                      # port busy, not found, ...
            self.failed.emit(f"cannot open source: {exc}")
            return
        try:
            pending, last_emit = [], time.monotonic()
            while not self.isInterruptionRequested():
                data = self.source.read()
                if data:
                    pending.extend(self.decoder.decode(f) for f in self.parser.feed(data))
                now = time.monotonic()
                if pending and now - last_emit >= BATCH_INTERVAL_S:
                    self.batch.emit(pending)
                    pending, last_emit = [], now
        except Exception as exc:
            self.failed.emit(f"read error: {exc}")
        finally:
            self.source.close()

    def stop(self) -> None:
        self.requestInterruption()
        self.wait(3000)
