from __future__ import annotations
import time
from typing import Optional, Tuple, List, Callable
import cv2
import numpy as np
import requests
from PyQt6.QtCore import QThread, pyqtSignal


def extract_host_from_stream_url(url: str) -> str:
    if not url:
        return ""
    hostpart = url.split("://", 1)[-1]
    hostpart = hostpart.split("/", 1)[0]
    host = hostpart.split(":", 1)[0]
    return host


class ESP32CamClient:
    def __init__(self, stream_url: str, timeout: float = 3.0):
        self.stream_url = stream_url
        self.timeout = timeout
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        self._cap = cv2.VideoCapture(self.stream_url)
        return bool(self._cap and self._cap.isOpened())

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if not self._cap:
            return False, None
        ok, frame = self._cap.read()
        if not ok:
            return False, None
        return True, frame

    def close(self):
        if self._cap:
            self._cap.release()
            self._cap = None

    def set_params(self, **params) -> bool:
        host = extract_host_from_stream_url(self.stream_url)
        if not host:
            return False
        url = f"http://{host}/set"
        try:
            requests.get(url, params=params, timeout=self.timeout)
            return True
        except Exception as e:
            print("[esp32cam.set_params]", e)
            return False


class CameraWorker(QThread):
    frameReady = pyqtSignal(np.ndarray)   # BGR 
    fpsReady   = pyqtSignal(float)
    opened     = pyqtSignal(bool, str)
    hazmatDet  = pyqtSignal(list)         # list[str]
    qrDet      = pyqtSignal(list)         # list[str]

    def __init__(self,
                 stream_url: str,
                 hazmat_factory: Optional[Callable[[], object]] = None,
                 qr_factory: Optional[Callable[[], object]] = None):
        super().__init__()
        self.client = ESP32CamClient(stream_url)
        self._running = False

        self.hazmat_factory = hazmat_factory
        self.qr_factory = qr_factory

        self.enable_hazmat = False
        self.enable_qr = False
        self.hazmat = None
        self.qr = None

    def run(self):
        self._running = True
        if not self.client.open():
            self.opened.emit(False, f"Could not open stream {self.client.stream_url}")
            return

        self.opened.emit(True, "opened")
        last = time.time()

        while self._running:
            ok, frame = self.client.read()
            if not ok or frame is None:
                self.msleep(10)
                continue

            haz_texts: List[str] = []
            qr_texts: List[str] = []

            if self.enable_hazmat and self.hazmat is None and self.hazmat_factory:
                try:
                    self.hazmat = self.hazmat_factory()
                except Exception as e:
                    print("[hazmat] init error:", e)
                    self.enable_hazmat = False

            if self.enable_qr and self.qr is None and self.qr_factory:
                try:
                    self.qr = self.qr_factory()
                except Exception as e:
                    print("[qr] init error:", e)
                    self.enable_qr = False

            # Hazmat
            if self.enable_hazmat and self.hazmat:
                try:
                    cids, confs, boxes = self.hazmat.detect(frame)
                    frame = self.hazmat.annotate(frame, cids, confs, boxes)
                    if hasattr(self.hazmat, "labels"):
                        haz_texts = [
                            f"{self.hazmat.labels[cid]} ({conf:.2f})"
                            for cid, conf in zip(cids, confs)
                            if 0 <= cid < len(self.hazmat.labels)
                        ]
                    else:
                        haz_texts = [f"id{int(cid)} ({confs[i]:.2f})" for i, cid in enumerate(cids)]
                except Exception as e:
                    print("[hazmat] run error:", e)

            # QR
            if self.enable_qr and self.qr:
                try:
                    qr_res = self.qr.detect(frame)
                    frame = self.qr.annotate(frame, qr_res)
                    qr_texts = [txt for txt, _ in qr_res if txt]
                except Exception as e:
                    print("[qr] run error:", e)

            # FPS
            now = time.time()
            dt = now - last
            fps = 1.0 / dt if dt > 0 else 0.0
            last = now

            self.hazmatDet.emit(haz_texts)
            self.qrDet.emit(qr_texts)
            self.frameReady.emit(frame)
            self.fpsReady.emit(fps)

        self.client.close()

    def stop(self):
        self._running = False
        self.wait(1000)
