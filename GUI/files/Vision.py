import cv2
import threading
import queue
import time
import logging
import os
import numpy as np
from tools.Camera.CameraBase import LogitechCamera, ESP32Camera
from pupil_apriltags import Detector

logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

logging.disable(logging.DEBUG)

detected_tags = []
# ========================== THREADS ==========================

def camera_capture_thread(cam: LogitechCamera, proc_q: queue.Queue, display_q: queue.Queue, exit_event: threading.Event):
    with cam:
        while not exit_event.is_set():
            frame = cam.read()
            # put into processing queue (for detector)
            try:
                if proc_q.full():
                    _ = proc_q.get_nowait()
            except queue.Empty:
                pass
            try:
                proc_q.put_nowait(frame.copy())
            except queue.Full:
                pass

            # put into display queue (for overlay/livefeed)
            try:
                if display_q.full():
                    _ = display_q.get_nowait()
            except queue.Empty:
                pass
            try:
                display_q.put_nowait(frame.copy())
            except queue.Full:
                pass

            time.sleep(0.01)
    logging.debug("Camera capture thread closed as exit_event detected.")


def tag_detector_thread(proc_q: queue.Queue, publish_tag: queue.Queue, tag_lock: threading.Lock, exit_event: threading.Event):
    param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
    camera_matrix = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), delimiter=',')
    dist_coeffs = np.loadtxt(os.path.join(param_dir, "distortion_coeffs.csv"), delimiter=',')
    tag_size = 0.015

    global detected_tags

    detector = Detector(families='tagStandard41h12', nthreads=1)
    while not exit_event.is_set():
        try:
            frame = proc_q.get(timeout=0.1)
        except queue.Empty:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]),
            tag_size=tag_size
        )
        with tag_lock:
            detected_tags.clear()
            detected_tags.extend(tags)

                # 拷贝并发布给外部进程/线程
        snapshot = [t for t in tags]  # 浅拷贝
        try:
            publish_tag.put_nowait(snapshot)
        except queue.Full:
            # 保持队列只存最新一次
            _ = publish_tag.get_nowait()
            publish_tag.put_nowait(snapshot)
        

    logging.debug("Tag Detector thread closed as exit_event detected.")

def overlay_thread(display_q: queue.Queue, overlaid_q: queue.Queue, uv_queue: queue.Queue, tag_lock: threading.Lock, exit_event: threading.Event):
    global detected_tags
    last_uv_list = None  # 记住最近一次的 [(id,u,v), ...]

    while not exit_event.is_set():
        try:
            frame = display_q.get(timeout=0.1)
        except queue.Empty:
            continue

        # 叠 Apriltag 边框/ID（原逻辑）
        with tag_lock:
            for tag in detected_tags:
                center = (int(tag.center[0]), int(tag.center[1]))
                cv2.circle(frame, center, 5, (0, 255, 0), -1)
                cv2.putText(frame, f"ID:{tag.tag_id}", (center[0] + 10, center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 取最新一包 uv（可能是 list，也可能是老格式 tuple）
        try:
            while True:
                last_uv_list = uv_queue.get_nowait()
        except queue.Empty:
            pass

        # 画 (id,u,v)
        if last_uv_list is not None:
            # 兼容老的 (u,v)
            if isinstance(last_uv_list, tuple) and len(last_uv_list) == 2:
                last_uv_list = [(-1, float(last_uv_list[0]), float(last_uv_list[1]))]

            # 给每个点上色并标注
            palette = [(255, 0, 255), (0, 128, 255), (255, 128, 0), (128, 255, 0)]
            for i, (tid, u, v) in enumerate(last_uv_list):
                u_i, v_i = int(u), int(v)
                color = palette[i % len(palette)]
                cv2.drawMarker(frame, (u_i, v_i), color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)
                cv2.putText(frame, f"({tid}) uv=({u_i},{v_i})", (u_i + 10, v_i - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 推给显示
        if overlaid_q.full():
            try: overlaid_q.get_nowait()
            except: pass
        try:
            overlaid_q.put_nowait(frame.copy())
        except queue.Full:
            pass

    logging.debug("Camera Overlay thread closed as exit_event detected.")




def livefeed_thread(overlaid_q: queue.Queue, exit_event: threading.Event):
    while not exit_event.is_set():
        try:
            frame = overlaid_q.get(timeout=0.1)
        except queue.Empty:
            continue
        cv2.imshow("Live Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_event.set()
            break
    cv2.destroyAllWindows()

import queue
import logging
import time
from typing import Optional, Sequence, List, Tuple

_LOG = logging.getLogger(__name__)

def _tag_area(tag) -> float:
    """Estimate tag area from corners (quadrilateral)."""
    try:
        c = np.array(tag.corners, dtype=float).reshape(-1, 2)
        # shoelace formula for polygon area
        x = c[:,0]; y = c[:,1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    except Exception:
        return 0.0

# 放在现有线程函数附近
def uv_publisher_thread(
    tag_lock: threading.Lock,
    uv_queue: queue.Queue,
    exit_event: threading.Event,
    preferred_tag_ids: Optional[Sequence[int]] = None,  # <- 支持多个ID
    publish_hz: float = 20.0,
    image_center: Optional[tuple] = None,
    max_tags: int = 2,                                  # <- 最多发布几个
):
    """
    发布最多 max_tags 个 (id,u,v)，按以下策略依次选：
      1) preferred_tag_ids 中出现的；
      2) 距离 image_center 最近的；
      3) 面积最大的。
    uv_queue 建议 maxsize=1，只保留最新列表。
    """
    dt = 1.0 / max(1.0, float(publish_hz))
    pref_ids = set(int(x) for x in preferred_tag_ids) if preferred_tag_ids else set()
    global detected_tags
    detected_tags_container = detected_tags

    while not exit_event.is_set():
        try:
            # 1) 快照
            tags_snapshot = list(detected_tags_container)

            if not tags_snapshot:
                time.sleep(dt)
                continue

            chosen: List = []

            # 工具函数
            def add_if_ok(t):
                if t is None: return
                if any(getattr(t, 'tag_id', None) == getattr(x, 'tag_id', None) for x in chosen):
                    return
                chosen.append(t)

            # 2.1 先挑偏好ID
            if pref_ids:
                for t in tags_snapshot:
                    try:
                        if int(t.tag_id) in pref_ids:
                            add_if_ok(t)
                            if len(chosen) >= max_tags: break
                    except Exception:
                        continue

            # 2.2 再按中心最近补满
            if len(chosen) < max_tags and image_center is not None:
                cx, cy = image_center
                cand = []
                for t in tags_snapshot:
                    try:
                        u, v = float(t.center[0]), float(t.center[1])
                        d = abs(u - cx) + abs(v - cy)  # L1
                        cand.append((d, t))
                    except Exception:
                        continue
                for _, t in sorted(cand, key=lambda x: x[0]):
                    if len(chosen) >= max_tags: break
                    add_if_ok(t)

            # 2.3 再按面积最大补满
            if len(chosen) < max_tags:
                cand = [(-_tag_area(t), t) for t in tags_snapshot]  # 负号→面积大排前
                for _, t in sorted(cand, key=lambda x: x[0]):
                    if len(chosen) >= max_tags: break
                    add_if_ok(t)

            # 3) 发布 [(id,u,v), ...]
            if chosen:
                payload: List[Tuple[int, float, float]] = []
                for t in chosen:
                    try:
                        payload.append((int(t.tag_id), float(t.center[0]), float(t.center[1])))
                    except Exception:
                        pass

                if payload:
                    try:
                        if uv_queue.full():
                            _ = uv_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        uv_queue.put_nowait(payload)  # 列表形式
                    except queue.Full:
                        pass

        except Exception as ex:
            _LOG.exception("uv_publisher_thread: unexpected error: %s", ex)
            time.sleep(0.05)

        time.sleep(dt)

def uv_adapter_thread(uv_list_q, uv_single_q, stop_event):
    """
    Read [(id,u,v), ...] and publish a representative (u,v) to uv_single_q
    (e.g., average of available points). Keeps IBVSReactor unchanged.
    """
    last = None
    while not stop_event.is_set():
        try:
            while True:
                last = uv_list_q.get_nowait()
        except queue.Empty:
            pass

        if last:
            # normalize payload
            pts = []
            if isinstance(last, tuple) and len(last) == 2:
                pts = [last]
            else:
                for item in last:
                    try:
                        _, u, v = item
                        pts.append((float(u), float(v)))
                    except Exception:
                        pass

            if pts:
                u = sum(p[0] for p in pts) / len(pts)
                v = sum(p[1] for p in pts) / len(pts)
                try:
                    if uv_single_q.full():
                        uv_single_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    uv_single_q.put_nowait((u, v))
                except queue.Full:
                    pass

        time.sleep(0.01)


# ========================== MAIN ==========================

if __name__ == '__main__':
    # cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    cam = ESP32Camera("http://172.20.10.2/")

    proc_q = queue.Queue(maxsize=2)      # for detector
    display_q = queue.Queue(maxsize=2)   # for overlay / live feed
    overlaid_q = queue.Queue(maxsize=2)
    publish_tag = queue.Queue(maxsize=4) # optional (和 uv publisher 无关)
    tag_lock = threading.Lock()

    detected_tags = []                   # ✅ 必须是 list
    uv_queue = queue.Queue(maxsize=1)    # ✅ 新增：存放 (u, v)
    exit_event = threading.Event()

    threads = [
        threading.Thread(target=camera_capture_thread, args=(cam, proc_q, display_q, exit_event), daemon=True),
        threading.Thread(target=tag_detector_thread, args=(proc_q, publish_tag, tag_lock, exit_event), daemon=True),

        # ✅ 新增：启动 uv_publisher_thread
        threading.Thread(
            target=uv_publisher_thread,
            args=(tag_lock, uv_queue, exit_event),
            kwargs=dict(preferred_tag_ids=[0, 1], publish_hz=30.0, image_center=(320, 240), max_tags=2),
            daemon=True
        ),

        threading.Thread(target=overlay_thread, args=(display_q, overlaid_q, uv_queue, tag_lock, exit_event), daemon=True),
        threading.Thread(target=livefeed_thread, args=(overlaid_q, exit_event), daemon=True)
    ]

    print("📡 Starting Vision Pipeline...")
    for t in threads: t.start()
    threads[-1].join()  # Wait for livefeed thread to quit (q pressed)
    exit_event.set()
    for t in threads: t.join()
    print("✅ Vision threads exited.")
