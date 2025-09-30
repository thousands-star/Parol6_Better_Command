import cv2
import threading
import queue
import time
import logging
import os
import numpy as np
from tools.Camera.CameraBase import CameraBase, LogitechCamera, ESP32Camera
from tools.tag_tools import _R_to_euler_xyz, _tag_object_corners
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


def tag_detector_thread(proc_q: queue.Queue, publish_tag: queue.Queue, cam: CameraBase, tag_lock: threading.Lock, exit_event: threading.Event):
    camera_matrix, dist_coeffs = cam.get_K()
    tag_size = 0.02

    global detected_tags

    detector = Detector(
        families='tagStandard41h12',
        nthreads=1,
        refine_edges=1,
        decode_sharpening=0.25
    )



    
    while not exit_event.is_set():
        try:
            frame = proc_q.get(timeout=0.1)
        except queue.Empty:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 先去畸变再检测
        # h, w = gray.shape[:2]
        # newK, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        # gray_u = cv2.undistort(gray, camera_matrix, dist_coeffs, None, newK)

        # tags = detector.detect(
        #     gray_u,
        #     estimate_tag_pose=False,
        #     camera_params=(newK[0, 0], newK[1, 1], newK[0, 2], newK[1, 2]),
        #     tag_size=tag_size
        # )

        tags = detector.detect(
            gray,
            estimate_tag_pose=False,
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

def overlay_thread(display_q: queue.Queue, overlaid_q: queue.Queue,
                   tag_lock: threading.Lock, exit_event: threading.Event):
    global detected_tags, detected_uvs, uv_lock

    def draw_text_outline(img, text, org, color=(255,255,255), outline_color=(0,0,0),
                          font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.45, thickness=1):
        # draw outline (thicker)
        cv2.putText(img, text, org, font, font_scale, outline_color, thickness=thickness+2, lineType=cv2.LINE_AA)
        # draw main text
        cv2.putText(img, text, org, font, font_scale, color, thickness=thickness, lineType=cv2.LINE_AA)

    while not exit_event.is_set():
        try:
            frame = display_q.get(timeout=0.1)
        except queue.Empty:
            continue

        # draw tags (centroid + id) as before
        with tag_lock:
            for tag in detected_tags:
                try:
                    center = (int(tag.center[0]), int(tag.center[1]))
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)
                    draw_text_outline(frame, f"ID:{tag.tag_id}", (center[0] + 10, center[1] - 8), color=(0,255,0))
                except Exception:
                    pass

        # read persistent detected_uvs
        try:
            uv_lock.acquire()
            if detected_uvs:
                last_uv_list = list(detected_uvs)
            else:
                last_uv_list = None
        finally:
            uv_lock.release()

        if last_uv_list is not None:
            palette = [(255, 0, 255), (0, 128, 255), (255, 128, 0), (128, 255, 0)]
            for i, entry in enumerate(last_uv_list):
                # support formats:
                # legacy: (tid, u, v)
                # corners: (tid, u, v, corners_list)
                try:
                    if isinstance(entry, tuple) and len(entry) == 3:
                        tid, u, v = entry
                        corners = None
                    elif isinstance(entry, tuple) and len(entry) == 4:
                        tid, u, v, corners = entry
                    else:
                        # fallback if dict-like
                        tid = entry.get('id', -1)
                        u, v = entry.get('center', (np.nan, np.nan))
                        corners = entry.get('corners', None)
                except Exception:
                    continue

                color = palette[i % len(palette)]
                u_i, v_i = int(round(u)), int(round(v))

                # draw center marker and id
                cv2.drawMarker(frame, (u_i, v_i), color,
                               markerType=cv2.MARKER_TILTED_CROSS, markerSize=14, thickness=2)
                draw_text_outline(frame, f"({int(tid)})", (u_i + 8, v_i - 10), color=color)

                # if corners present, draw polygon + corner markers & print coords
                if corners:
                    try:
                        pts = np.array(corners, dtype=np.int32)
                        # polygon
                        if pts.ndim == 2 and pts.shape[0] >= 3:
                            cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)

                        # corner markers and coordinate text for each corner
                        # corners order kept as returned by detector: index 0..3
                        for ci, (cx, cy) in enumerate(corners):
                            cx_i, cy_i = int(round(cx)), int(round(cy))
                            cv2.circle(frame, (cx_i, cy_i), 4, color, -1)
                            # print index near corner
                            draw_text_outline(frame, f"{ci}", (cx_i + 4, cy_i + 4), color=(255,255,255))
                            # print coordinate (x,y) slightly offset (below the point)
                            coord_text = f"{cx_i},{cy_i}"
                            # offset each corner's coord placement to avoid overlap:
                            offs = [(6,18), (6,18), (6,-6), (6,-6)]
                            dx, dy = offs[ci % len(offs)]
                            draw_text_outline(frame, coord_text, (cx_i + dx, cy_i + dy), color=(255,255,255), font_scale=0.4)

                        # also print a compact one-line summary at box top-left
                        min_x = int(np.min(pts[:,0]))
                        min_y = int(np.min(pts[:,1]))
                        # build compact string 'c0:(x,y) c1:(x,y) ...'
                        compact = " ".join([f"c{ci}:({int(round(corners[ci][0]))},{int(round(corners[ci][1]))})" for ci in range(len(corners))])
                        # fit text by truncating if too long
                        if len(compact) > 80:
                            compact = compact[:77] + "..."
                        draw_text_outline(frame, compact, (max(min_x, 2), max(min_y - 6, 12)), color=color, font_scale=0.45)
                    except Exception:
                        pass

            # show top2 centers text (compatible)
            top2 = last_uv_list[:2]
            uv_line_elems = []
            for entry in top2:
                if isinstance(entry, tuple) and len(entry) >= 3:
                    uv_line_elems.append(f"({int(round(entry[1]))},{int(round(entry[2]))})")
                else:
                    uv_line_elems.append("(?,?)")
            uv_line = "  ".join(uv_line_elems)
            draw_text_outline(frame, f"UV: {uv_line}", (10, 28), color=(0,255,255), font_scale=0.6)

        # push to display queue
        if overlaid_q.full():
            try:
                overlaid_q.get_nowait()
            except:
                pass
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

def uv_publisher_thread(
    tag_lock: threading.Lock,
    publish_uv: queue.Queue,
    exit_event: threading.Event,
    preferred_tag_ids: Optional[Sequence[int]] = None,
    publish_hz: float = 2.0,
    cam: CameraBase = None,
    max_tags: int = 2,
    publish_corners: bool = True,
):
    """
    从全局 detected_tags 中挑 tag，并：
      1) 把 compact payload 写入 detected_uvs（供 overlay 使用）
      2) 非阻塞地把 snapshot 放到 publish_uv
      3) 把 tag center + corners 以可读格式写到日志（INFO）
    日志只在 payload 发生变化时输出（并且最低间隔 log_interval 秒）。
    """
    dt = 1.0 / max(1.0, float(publish_hz))
    pref_ids = set(int(x) for x in preferred_tag_ids) if preferred_tag_ids else set()
    global detected_tags, detected_uvs, uv_lock

    last_payload = None
    last_log_ts = 0.0
    log_interval = max(0.05, min(1.0, 1.0 / max(1.0, float(publish_hz))))  # 最低间隔，防刷屏
    if cam is not None:
        camera_matrix, dist_coeff = cam.get_K()
        image_center = (camera_matrix[0, 2], camera_matrix[1, 2])
    else:
        image_center = None

    while not exit_event.is_set():
        try:
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
                    tag_id = int(t.tag_id)
                    cx = float(t.center[0])
                    cy = float(t.center[1])
                    if publish_corners:
                        corners = [(float(c[0]), float(c[1])) for c in t.corners]
                        payload.append((tag_id, cx, cy, corners))
                    else:
                        payload.append((tag_id, cx, cy))
                except Exception:
                    pass

            if payload:
                # persistent store for overlay
                try:
                    uv_lock.acquire()
                    detected_uvs.clear()
                    detected_uvs.extend(payload)
                finally:
                    uv_lock.release()

                # publish snapshot non-blocking
                try:
                    if publish_uv.full():
                        _ = publish_uv.get_nowait()
                except queue.Empty:
                    pass
                try:
                    publish_uv.put_nowait(list(payload))
                except queue.Full:
                    pass

                # ---- logging: only when changed or after interval ----
                now = time.time()
                if payload != last_payload and (now - last_log_ts) >= log_interval:
                    # build human-friendly multi-line message
                    lines = []
                    for item in payload:
                        if isinstance(item, tuple) and len(item) == 4:
                            tid, cx, cy, corners = item
                            corners_str = ", ".join([f"({int(round(x))},{int(round(y))})" for x,y in corners])
                            lines.append(f"Tag {tid}: center=({int(round(cx))},{int(round(cy))}) corners=[{corners_str}]")
                        elif isinstance(item, tuple) and len(item) == 3:
                            tid, cx, cy = item
                            lines.append(f"Tag {tid}: center=({int(round(cx))},{int(round(cy))})")
                        else:
                            lines.append(str(item))
                    # _LOG.info("uv_publisher: %s", " | ".join(lines))
                    last_payload = payload
                    last_log_ts = now

        except Exception as ex:
            _LOG.exception("uv_publisher_thread: unexpected error: %s", ex)
            time.sleep(0.05)

        time.sleep(dt)

def pose_from_two_tags_thread(
    publish_uv: queue.Queue,
    pose_out: queue.Queue,
    exit_event: threading.Event,
    K: np.ndarray,
    dist: np.ndarray,
    tag_size: float,
    required_ids=(0, 1),
    center_map=None,
    refine=True,
    fps=20.0,
):
    """
    从 publish_uv 消费两块 tag 的 8 个像素角点，结合已知物体平面几何，用 solvePnP 求相机位姿。
    - required_ids: 需要同时出现的两个 tag 的 id（默认 0 和 1）
    - center_map: 物体系下每个 tag 的中心 (x, y)（米）。若不给，默认 id0 在 (0,0)，id1 在 (d,0)
    - pose_out: 输出队列（maxsize=1），元素形如 dict(rx,ry,rz,rvec,tvec,R,stamp)
    """

    dt = 1.0 / max(1.0, float(fps))
    req = tuple(int(x) for x in required_ids)
    # 默认两块中心相距 6 cm（沿 +X）
    if center_map is None:
        d = 0.060  # 0.06 m
        center_map = {req[0]: (0.0, 0.0), req[1]: (d, 0.0)}

    last_pose = None
    while not exit_event.is_set():
        # 拿到最新一帧 payload（形式：[ (id, cx, cy, corners), ... ]）
        try:
            payload = publish_uv.get(timeout=dt)
            # 把队列里旧的清空，只保留最新
            while not publish_uv.empty():
                payload = publish_uv.get_nowait()
        except queue.Empty:
            time.sleep(dt)
            continue

        # 收集所需 id 的四角
        got = {}
        for item in payload:
            # 既兼容 (tid, cx, cy, corners) 也允许 dict 形式
            if isinstance(item, tuple) and len(item) == 4:
                tid, _, _, corners = item
            else:
                tid = int(item.get("id", -1))
                corners = item.get("corners", None)
            if tid in req and corners is not None and len(corners) >= 4:
                # corners 已在 overlay 线程以 detector 的顺序 (0..3) 输出了
                got[tid] = np.array(corners[:4], dtype=np.float32)

        if len(got) < 2:
            time.sleep(dt)
            continue

        # 2D-3D 对应点
        obj_pts = []
        img_pts = []
        try:
            for tid in req:
                cxy = center_map[tid]  # (x, y) in meters
                obj_pts.append(_tag_object_corners(cxy, tag_size))  # (4,3)
                img_pts.append(got[tid])                             # (4,2)
            obj_pts = np.vstack(obj_pts).astype(np.float32)  # (8,3)
            img_pts = np.vstack(img_pts).astype(np.float32)  # (8,2)
        except KeyError:
            # center_map 不完整
            time.sleep(dt)
            continue

        # PnP 求解
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            time.sleep(dt)
            continue
        if refine and hasattr(cv2, "solvePnPRefineLM"):
            rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, K, dist, rvec, tvec)

        R, _ = cv2.Rodrigues(rvec)
        rx, ry, rz = _R_to_euler_xyz(R, degrees=True)
        pose = dict(rx=rx, ry=ry, rz=rz, rvec=rvec.reshape(3), tvec=tvec.reshape(3), R=R, stamp=time.time())
        last_pose = pose

        # 非阻塞更新输出（只保留最新）
        try:
            if pose_out.full():
                _ = pose_out.get_nowait()
        except queue.Empty:
            pass
        try:
            pose_out.put_nowait(pose)
        except queue.Full:
            pass

        # 也可以打印一行看看
        print(f"[POSE] rx={rx:.2f} ry={ry:.2f} rz={rz:.2f}  t={tvec.reshape(3)}")

        time.sleep(dt)
# ========================== MAIN ==========================

if __name__ == '__main__':

    # param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
    # K = np.loadtxt(os.path.join(param_dir, "webcam_camera_intrinsic_matrix.csv"), delimiter=',').astype(np.float32)
    # dist = np.loadtxt(os.path.join(param_dir, "webcam_distortion_coeffs.csv"), delimiter=',').astype(np.float32)
    # cam = LogitechCamera(source=1, width=640, height=480, fps=30,K=K, dist=dist)
    
    param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
    K = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), delimiter=',').astype(np.float32)
    dist = np.loadtxt(os.path.join(param_dir, "distortion_coeffs.csv"), delimiter=',').astype(np.float32)
    cam = ESP32Camera("http://172.20.10.2/",K=K, dist=dist)

    proc_q = queue.Queue(maxsize=2)      # for detector
    display_q = queue.Queue(maxsize=2)   # for overlay / live feed
    overlaid_q = queue.Queue(maxsize=2)
    publish_tag = queue.Queue(maxsize=4) # optional (和 uv publisher 无关)
    publish_uv = queue.Queue(maxsize=4)   # 对外发布队列（外面消费不会影响 detected_uvs）
    tag_lock = threading.Lock()

    detected_tags = []                   # ✅ 必须是 list
    uv_queue = queue.Queue(maxsize=1)    # ✅ 新增：存放 (u, v)
    exit_event = threading.Event()



    tag_size = 0.02  # ★ 修改成你的实际边长（米）
    # ★ 告诉线程两个 tag 在物体平面上的中心位置（米）。例：id 0 在原点，id 1 在 +X 方向 60 mm
    center_map = {2: (0.0, 0.0), 5: (-0.090, 0.0)}
    center_map = {6: (0.0, 0.0), 9: (-0.030, 0.0)}

    preferred_id = [2,5]
    preferred_id = [6,9]
    pose_out = queue.Queue(maxsize=1)

    threads = [
        threading.Thread(target=camera_capture_thread, args=(cam, proc_q, display_q, exit_event), daemon=True),
        threading.Thread(target=tag_detector_thread,    args=(proc_q, publish_tag, cam, tag_lock, exit_event), daemon=True),
        threading.Thread(
            target=uv_publisher_thread,
            args=(tag_lock, publish_uv, exit_event),
            kwargs=dict(preferred_tag_ids=preferred_id, publish_hz=30.0, cam = cam, max_tags=2),
            daemon=True
        ),
        threading.Thread(
            target=pose_from_two_tags_thread,
            args=(publish_uv, pose_out, exit_event, K, dist, tag_size),
            kwargs=dict(required_ids=preferred_id, center_map=center_map, refine=True, fps=30.0),
            daemon=True
        ),
        threading.Thread(target=overlay_thread, args=(display_q, overlaid_q, tag_lock, exit_event), daemon=True),
        threading.Thread(target=livefeed_thread, args=(overlaid_q, exit_event), daemon=True),
    ]

    print("📡 Starting Vision Pipeline...")
    for t in threads: t.start()

    # 简单的测试消费者：每 0.3 s 打印一次最新欧拉角
    try:
        while not exit_event.is_set():
            try:
                p = pose_out.get(timeout=0.3)
                print(f"[POSE] rx={p['rx']:.2f}° ry={p['ry']:.2f}° rz={p['rz']:.2f}°  t={p['tvec']}")
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        exit_event.set()

    threads[-1].join()
    exit_event.set()
    for t in threads: t.join()
    print("✅ Vision threads exited.")
