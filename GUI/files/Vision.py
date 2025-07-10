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
    import numpy as np

    node = CamVisionNode(cam_id=0, width=640, height=480, fps=30)
    try:
        node.start()
        print("[INFO] CamVisionNode started. Press 'q' to quit.")
        while True:
            frame = node.latest_rgb            # 最新 BGR 帧
            if frame is None:
                time.sleep(0.01)
                continue

            # 取一份可写副本
            vis = frame.copy()

            # 把检测结果画出来
            for tag in node.latest_tag_poses:
                R, t = tag.R, tag.t            # 旋转矩阵 & 平移向量
                c = (0, 255, 0)                # 绿色框
                # pupil-apriltags 的 detection 里会有 corner 坐标
                # 这里重新跑一次 detect() 取 corner，也可以把 corner 存到 TagPose 里
                # 为了演示简单，直接用 detector 重新 detect：
            # --------------------------------------------------
            # **注意**：下面这一步只是为了拿到 corner，如果你在 _infer_loop
            #           里把 t.corners 塞进 TagPose，就不用再 detect 一次了
            # --------------------------------------------------
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                dets = node.tag_detector.detect(gray)
                for d in dets:
                    if d.tag_id != tag.id:
                        continue
                    corners = d.corners.astype(int)
                    for i in range(4):
                        pt1 = tuple(corners[i])
                        pt2 = tuple(corners[(i + 1) % 4])
                        cv2.line(vis, pt1, pt2, c, 2)

                    # 在 tag 中心写 ID
                    center = tuple(d.center.astype(int))
                    cv2.putText(
                        vis, f"id:{tag.id}",
                        center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
                    )
                    # 距离/姿态简单展示一下 Z 轴位移
                    z_mm = t[2][0] * 1000     # 假设单位是米 → 毫米
                    cv2.putText(
                        vis, f"Z={z_mm:.1f}mm",
                        (center[0], center[1] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
                    )

            # show
            cv2.imshow("Apriltag Vision", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] Shutting down...")
        node.stop()
        cv2.destroyAllWindows()
