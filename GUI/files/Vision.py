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
detected_uvs = []         
uv_lock = threading.Lock()  
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

def overlay_thread(display_q: queue.Queue, overlaid_q: queue.Queue, tag_lock: threading.Lock, exit_event: threading.Event):
    global detected_tags, detected_uvs, uv_lock
    last_uv_list = None

    while not exit_event.is_set():
        try:
            frame = display_q.get(timeout=0.1)
        except queue.Empty:
            continue

        # draw tags (same as before)
        with tag_lock:
            for tag in detected_tags:
                center = (int(tag.center[0]), int(tag.center[1]))
                cv2.circle(frame, center, 5, (0, 255, 0), -1)
                cv2.putText(frame, f"ID:{tag.tag_id}", (center[0] + 10, center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # === 从 detected_uvs 读取（不会被外面消费掉） ===
        try:
            uv_lock.acquire()
            if detected_uvs:
                last_uv_list = list(detected_uvs)   # 拷贝一份用于绘制
            else:
                last_uv_list = None
        finally:
            uv_lock.release()

        # 兼容老格式 tuple (u,v)
        if last_uv_list is not None:
            if isinstance(last_uv_list, tuple) and len(last_uv_list) == 2:
                last_uv_list = [(-1, float(last_uv_list[0]), float(last_uv_list[1]))]

            palette = [(255, 0, 255), (0, 128, 255), (255, 128, 0), (128, 255, 0)]
            for i, (tid, u, v) in enumerate(last_uv_list):
                u_i, v_i = int(u), int(v)
                color = palette[i % len(palette)]
                cv2.drawMarker(frame, (u_i, v_i), color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)
                cv2.putText(frame, f"({tid}) uv=({u_i},{v_i})", (u_i + 10, v_i - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # push to display queue
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
    publish_uv: queue.Queue,
    exit_event: threading.Event,
    preferred_tag_ids: Optional[Sequence[int]] = None,
    publish_hz: float = 20.0,
    image_center: Optional[tuple] = None,
    max_tags: int = 2,
):
    """
    从全局 detected_tags 中挑出最多 max_tags 个点，
    写入全局 detected_uvs（加锁），并把快照放到 publish_uv（供外部进程/线程消费）。
    """
    dt = 1.0 / max(1.0, float(publish_hz))
    pref_ids = set(int(x) for x in preferred_tag_ids) if preferred_tag_ids else set()
    global detected_tags, detected_uvs, uv_lock

    while not exit_event.is_set():
        try:
            # snapshot tags (no lock on tag list here because tag lock already protects modification)
            with tag_lock:
                tags_snapshot = list(detected_tags)

            if not tags_snapshot:
                time.sleep(dt)
                continue

            chosen = []
            def add_if_ok(t):
                if t is None: return
                if any(getattr(t, 'tag_id', None) == getattr(x, 'tag_id', None) for x in chosen):
                    return
                chosen.append(t)

            # 1) preferred ids
            if pref_ids:
                for t in tags_snapshot:
                    try:
                        if int(t.tag_id) in pref_ids:
                            add_if_ok(t)
                            if len(chosen) >= max_tags: break
                    except Exception:
                        continue

            # 2) fill by distance to center
            if len(chosen) < max_tags and image_center is not None:
                cx, cy = image_center
                cand = []
                for t in tags_snapshot:
                    try:
                        u, v = float(t.center[0]), float(t.center[1])
                        d = abs(u - cx) + abs(v - cy)
                        cand.append((d, t))
                    except Exception:
                        continue
                for _, t in sorted(cand, key=lambda x: x[0]):
                    if len(chosen) >= max_tags: break
                    add_if_ok(t)

            # 3) fill by area
            if len(chosen) < max_tags:
                cand = [(-_tag_area(t), t) for t in tags_snapshot]
                for _, t in sorted(cand, key=lambda x: x[0]):
                    if len(chosen) >= max_tags: break
                    add_if_ok(t)

            # build payload
            payload = []
            for t in chosen:
                try:
                    payload.append((int(t.tag_id), float(t.center[0]), float(t.center[1])))
                except Exception:
                    pass

            if payload:
                # write to persistent list (detected_uvs) under lock
                try:
                    uv_lock.acquire()
                    detected_uvs.clear()
                    detected_uvs.extend(payload)   # 持久存储最近一次列表（不会被外部消费掉）
                finally:
                    uv_lock.release()

                # also publish a snapshot to publish_uv for external consumers (non-blocking)
                try:
                    if publish_uv.full():
                        _ = publish_uv.get_nowait()
                except queue.Empty:
                    pass
                try:
                    publish_uv.put_nowait(list(payload))
                except queue.Full:
                    pass

        except Exception as ex:
            _LOG.exception("uv_publisher_thread: unexpected error: %s", ex)
            time.sleep(0.05)

        time.sleep(dt)


# ========================== MAIN ==========================

if __name__ == '__main__':
    # cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    cam = ESP32Camera("http://172.20.10.2/")

    proc_q = queue.Queue(maxsize=2)      # for detector
    display_q = queue.Queue(maxsize=2)   # for overlay / live feed
    overlaid_q = queue.Queue(maxsize=2)
    publish_tag = queue.Queue(maxsize=4) # optional (和 uv publisher 无关)
    publish_uv = queue.Queue(maxsize=4)   # 对外发布队列（外面消费不会影响 detected_uvs）
    tag_lock = threading.Lock()

    detected_tags = []                   # ✅ 必须是 list
    uv_queue = queue.Queue(maxsize=1)    # ✅ 新增：存放 (u, v)
    exit_event = threading.Event()

    threads = [
        threading.Thread(target=camera_capture_thread, args=(cam, proc_q, display_q, exit_event), daemon=True),
        threading.Thread(target=tag_detector_thread, args=(proc_q, publish_tag, tag_lock, exit_event), daemon=True),

        threading.Thread(
            target=uv_publisher_thread,
            args=(tag_lock, publish_uv, exit_event),
            kwargs=dict(preferred_tag_ids=[0, 1], publish_hz=30.0, image_center=(320, 240), max_tags=2),
            daemon=True
        ),

        threading.Thread(target=overlay_thread, args=(display_q, overlaid_q, tag_lock, exit_event), daemon=True),
        threading.Thread(target=livefeed_thread, args=(overlaid_q, exit_event), daemon=True)
    ]

    print("📡 Starting Vision Pipeline...")
    for t in threads: t.start()
    threads[-1].join()  # Wait for livefeed thread to quit (q pressed)
    exit_event.set()
    for t in threads: t.join()
    print("✅ Vision threads exited.")
