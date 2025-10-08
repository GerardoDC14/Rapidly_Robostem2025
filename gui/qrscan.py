from __future__ import annotations
import cv2
import numpy as np
from typing import List, Tuple

class QRScanner:
    def __init__(self):
        self.det = cv2.QRCodeDetector()

    def detect(self, frame) -> List[Tuple[str, np.ndarray]]:
        try:
            # Nuevo OpenCV: 4 values
            retval, decoded_info, decoded_points, _ = self.det.detectAndDecodeMulti(frame)
            texts = decoded_info if retval else []
            points = decoded_points if retval else None
        except ValueError:
            # Viejo OpenCV: 3 values
            decoded_info, decoded_points, _ = self.det.detectAndDecodeMulti(frame)
            texts = decoded_info
            points = decoded_points

        out: List[Tuple[str, np.ndarray]] = []
        if points is not None and len(points) > 0:
            n = min(len(points), len(texts) if texts is not None else 0)
            for i in range(n):
                txt = texts[i] if texts is not None else ""
                pts = np.array(points[i], dtype=np.float32)
                out.append((txt, pts))
        return out

    def annotate(self, frame, results: List[Tuple[str, np.ndarray]]):
        for txt, pts in results:
            if pts is None or len(pts) == 0:
                continue
            pts = pts.astype(int).reshape(-1, 2)
            for j in range(len(pts)):
                p1, p2 = tuple(pts[j]), tuple(pts[(j+1) % len(pts)])
                cv2.line(frame, p1, p2, (0, 255, 255), 2)
            if txt:
                x, y = pts[0]
                cv2.putText(frame, txt, (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame
