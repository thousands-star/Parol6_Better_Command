import cv2
import threading
import queue
import time
import os
import numpy as np
from tools.Camera.CameraBase import LogitechCamera
from pupil_apriltags import Detector

# ========================== THREADS ==========================

def camera_capture_thread(cam: LogitechCamera, frame_q: queue.Queue, exit_event: threading.Event):
    with cam:
        while not exit_event.is_set():
            frame = cam.read()
            if frame_q.full():
                try: frame_q.get_nowait()
                except: pass
            try:
                frame_q.put_nowait(frame.copy())
            except queue.Full:
                pass
            time.sleep(0.01)

def tag_detector_thread(frame_q: queue.Queue, detected_tags: list, tag_lock: threading.Lock, exit_event: threading.Event):
    param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
    camera_matrix = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), delimiter=',')
    dist_coeffs = np.loadtxt(os.path.join(param_dir, "distortion_coeffs.csv"), delimiter=',')
    tag_size = 0.05

    detector = Detector(families='tagStandard41h12', nthreads=1)
    while not exit_event.is_set():
        try:
            frame = frame_q.get(timeout=0.1)
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

def overlay_thread(frame_q: queue.Queue, display_q: queue.Queue, detected_tags: list, tag_lock: threading.Lock, exit_event: threading.Event):
    while not exit_event.is_set():
        try:
            frame = frame_q.get(timeout=0.1)
        except queue.Empty:
            continue
        with tag_lock:
            for tag in detected_tags:
                center = (int(tag.center[0]), int(tag.center[1]))
                cv2.circle(frame, center, 5, (0, 255, 0), -1)
                cv2.putText(frame, f"ID:{tag.tag_id}", (center[0] + 10, center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                x, y, z = tag.pose_t.flatten()
                print(f"🧭 Tag ID {tag.tag_id} distance: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")
        if display_q.full():
            try: display_q.get_nowait()
            except: pass
        try:
            display_q.put_nowait(frame.copy())
        except queue.Full:
            pass

def livefeed_thread(display_q: queue.Queue, exit_event: threading.Event):
    while not exit_event.is_set():
        try:
            frame = display_q.get(timeout=0.1)
        except queue.Empty:
            continue
        cv2.imshow("Live Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_event.set()
            break
    cv2.destroyAllWindows()

# ========================== MAIN ==========================

if __name__ == '__main__':
    cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    frame_q = queue.Queue(maxsize=2)
    display_q = queue.Queue(maxsize=2)
    tag_lock = threading.Lock()
    detected_tags = []
    exit_event = threading.Event()

    threads = [
        threading.Thread(target=camera_capture_thread, args=(cam, frame_q, exit_event), daemon=True),
        threading.Thread(target=tag_detector_thread, args=(frame_q, detected_tags, tag_lock, exit_event), daemon=True),
        threading.Thread(target=overlay_thread, args=(frame_q, display_q, detected_tags, tag_lock, exit_event), daemon=True),
        threading.Thread(target=livefeed_thread, args=(display_q, exit_event), daemon=True)
    ]

    print("📡 Starting Vision Pipeline...")
    for t in threads: t.start()
    threads[-1].join()  # Wait for livefeed thread to quit (q pressed)
    exit_event.set()
    for t in threads: t.join()
    print("✅ Vision threads exited.")
