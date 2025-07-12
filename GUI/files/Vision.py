import cv2
import threading
import queue
import time
from tools.Camera.CameraBase import LogitechCamera
from pupil_apriltags import Detector

# 两个队列：一个供 detect 用，一个供显示用
frame_queue = queue.Queue(maxsize=1)
display_queue = queue.Queue(maxsize=1)

tag_lock = threading.Lock()
detected_tags = []
exit_event = threading.Event()

def camera_capture_thread(cam: LogitechCamera, frame_q: queue.Queue, display_q: queue.Queue):
    with cam:
        while not exit_event.is_set():
            frame = cam.read()
            for q in [frame_q, display_q]:
                if q.full():
                    q.get()
                q.put(frame.copy())
            time.sleep(0.02)

def tag_detector_thread(frame_q: queue.Queue):
    import numpy as np
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    param_dir = os.path.join(current_dir, "tools", "Camera", "param")
    camera_matrix = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), delimiter=',')
    dist_coeffs = np.loadtxt(os.path.join(param_dir, "distortion_coeffs.csv"), delimiter=',')
    tag_size = 0.05

    detector = Detector(families='tagStandard41h12', nthreads=1)
    while not exit_event.is_set():
        if not frame_q.empty():
            frame = frame_q.get()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2]),
                tag_size=tag_size
            )
            with tag_lock:
                detected_tags.clear()
                detected_tags.extend(tags)
        else:
            time.sleep(0.01)

def livefeed_thread(display_q: queue.Queue):
    while not exit_event.is_set():
        if not display_q.empty():
            frame = display_q.get()
            with tag_lock:
                for tag in detected_tags:
                    center = (int(tag.center[0]), int(tag.center[1]))
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID:{tag.tag_id}", (center[0]+10, center[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    x, y, z = tag.pose_t.flatten()
                    print(f"🧭 Tag ID {tag.tag_id} distance: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")
            cv2.imshow("Live Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit_event.set()
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    frame_q = queue.Queue(maxsize=1)
    display_q = queue.Queue(maxsize=1)

    t_cap = threading.Thread(target=camera_capture_thread, args=(cam, frame_q, display_q), daemon=True)
    t_det = threading.Thread(target=tag_detector_thread, args=(frame_q,), daemon=True)
    t_show = threading.Thread(target=livefeed_thread, args=(display_q,), daemon=True)

    print("📡 Starting 3-thread pipeline...")
    t_cap.start()
    t_det.start()
    t_show.start()

    t_show.join()
    exit_event.set()
    t_cap.join()
    t_det.join()
    print("✅ All threads exited cleanly.")

