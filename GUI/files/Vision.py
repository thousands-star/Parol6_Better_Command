# cam_vision_node.py
# -*- coding: utf-8 -*-
import cv2, threading, queue, time
from pupil_apriltags import Detector    # pip install pupil-apriltags
from collections import namedtuple

TagPose = namedtuple('TagPose', 'id R t')

class CamVisionNode:
    """
    单进程，两线程：Capture ↔ Infer
    对外暴露: start(), stop(), latest_rgb, latest_tag_poses
    """

    def __init__(
        self,
        cam_id: int = 0,
        queue_size: int = 2,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        self.tag_detector = Detector(families='tag36h11')

        self.frame_q: queue.Queue = queue.Queue(maxsize=queue_size)
        self.shutdown_evt = threading.Event()

        self.latest_rgb = None            # 最近一帧（BGR）
        self.latest_tag_poses: list[TagPose] = []
        self.t_cap = threading.Thread(target=self._capture_loop, daemon=True)
        self.t_inf = threading.Thread(target=self._infer_loop,   daemon=True)

    # ---------------- 开关 ----------------
    def start(self):
        self.shutdown_evt.clear()
        self.t_cap.start()
        self.t_inf.start()

    def stop(self):
        self.shutdown_evt.set()
        self.t_cap.join()
        self.t_inf.join()
        self.cap.release()

    # -------------- 线程函数 ---------------
    def _capture_loop(self):
        while not self.shutdown_evt.is_set():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01); continue
            self.latest_rgb = frame
            # 若队列已满，丢掉旧帧
            if self.frame_q.full():
                try: self.frame_q.get_nowait()
                except queue.Empty: pass
            self.frame_q.put(frame)

    def _infer_loop(self):
        while not self.shutdown_evt.is_set():
            try:
                frame = self.frame_q.get(timeout=0.05)
            except queue.Empty:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = self.tag_detector.detect(gray, estimate_tag_pose=True,
                                            camera_params=(600, 600, 320, 240),
                                            tag_size=0.04)
            tag_poses = [TagPose(t.tag_id, t.pose_R, t.pose_t) for t in tags]
            self.latest_tag_poses = tag_poses

if __name__ == "__main__":
    