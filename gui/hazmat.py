from __future__ import annotations
import cv2
import numpy as np
from typing import List, Tuple

class HazmatYOLO:
    def __init__(self,
                 cfg_path: str,
                 weights_path: str,
                 labels_path: str,
                 input_size=(416, 416),
                 conf_thresh=0.8,
                 nms_thresh=0.4,
                 backend="opencv"):
        self.cfg = cfg_path
        self.weights = weights_path
        self.labels = self._load_labels(labels_path)
        self.input_size = tuple(input_size)
        self.conf_thresh = float(conf_thresh)
        self.nms_thresh  = float(nms_thresh)

        self.model = cv2.dnn_DetectionModel(self.cfg, self.weights)
        self.model.setInputParams(scale=1/255.0, size=self.input_size, swapRB=True)

        if backend == "opencv":
            self.model.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        np.random.seed(42)
        self.colors = np.random.randint(0, 255, (max(1,len(self.labels)), 3), dtype="uint8")

    def _load_labels(self, p: str) -> List[str]:
        with open(p, "r") as f:
            return [ln.strip() for ln in f if ln.strip()]

    def detect(self, frame) -> Tuple[List[int], List[float], List[tuple]]:
        classIDs, confidences, boxes = self.model.detect(frame, self.conf_thresh, self.nms_thresh)
        cids = list(classIDs) if classIDs is not None else []
        confs = [float(c) for c in (confidences if confidences is not None else [])]
        bxs   = [tuple(b) for b in (boxes if boxes is not None else [])]
        return cids, confs, bxs

    def annotate(self, frame, cids, confs, boxes):
        H, W = frame.shape[:2]
        for cid, conf, box in zip(cids, confs, boxes):
            x, y, w, h = box
            color = [int(c) for c in self.colors[int(cid) % len(self.colors)]]
            name = self.labels[int(cid)] if 0 <= cid < len(self.labels) else f"id{cid}"
            cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
            cv2.putText(frame, f"{name}: {conf:.2f}", (x, max(15, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cx, cy = int(x+w/2), int(y+h/2)
            cv2.circle(frame, (cx,cy), 4, color, -1)
            cv2.putText(frame, f"({cx},{cy}) [{cx/W:.2f},{cy/H:.2f}]",
                        (x, min(H-5, y+h+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        return frame
