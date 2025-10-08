from __future__ import annotations
import json, time
import requests
from PyQt6.QtCore import QThread, pyqtSignal


class MagnetometerWorker(QThread):
    reading = pyqtSignal(float, float, float)   # x, y, z en uT
    rate    = pyqtSignal(float)                 # suavizado Hz
    status  = pyqtSignal(bool, str)             # (ok, message)

    def __init__(self, url: str, timeout_connect=3.0, timeout_read=30.0):
        super().__init__()
        self.url = url
        self.timeout_connect = timeout_connect
        self.timeout_read = timeout_read
        self._running = False

    def run(self):
        self._running = True
        ema = None
        t0 = time.time()
        cnt = 0

        while self._running:
            try:
                with requests.Session() as s:
                    self.status.emit(False, "connecting…")
                    with s.get(self.url, stream=True,
                               timeout=(self.timeout_connect, self.timeout_read)) as r:
                        r.raise_for_status()
                        self.status.emit(True, "connected")
                        t0 = time.time(); cnt = 0; ema = None

                        for raw in r.iter_lines(decode_unicode=True, delimiter=b"\n"):
                            if not self._running:
                                break
                            if not raw:
                                continue
                            try:
                                data = json.loads(raw)
                                x = float(data.get("x", 0.0))
                                y = float(data.get("y", 0.0))
                                z = float(data.get("z", 0.0))
                                self.reading.emit(x, y, z)

                                cnt += 1
                                now = time.time()
                                if now - t0 >= 1.0:
                                    inst = cnt / (now - t0)
                                    ema = inst if ema is None else (0.85 * ema + 0.15 * inst)
                                    self.rate.emit(ema)
                                    t0 = now
                                    cnt = 0
                            except Exception:
                                continue
            except Exception as e:
                self.status.emit(False, f"reconnecting… ({e})")
                for _ in range(5):
                    if not self._running: break
                    time.sleep(0.5)

    def stop(self):
        self._running = False
        self.wait(1500)
