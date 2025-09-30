import os
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import importlib

# === 动态选择相机类 ===
def get_camera(class_name: str, **kwargs):
    """
    动态导入并实例化相机类
    :param class_name: 字符串, 比如 'LogitechCamera' 或 'ESP32Camera'
    :param kwargs: 相机初始化参数
    :return: 相机对象
    """
    # 假设 CameraBase.py 里有所有的相机类（或对应模块可被 import）
    cam_module = importlib.import_module("CameraBase")
    cam_class = getattr(cam_module, class_name)
    return cam_class(**kwargs)

def visualize_reprojection_errors(images, objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    for i in range(len(images)):
        img = cv2.imread(images[i])
        img_display = img.copy()

        detected = imgpoints[i].reshape(-1, 2)
        reprojected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        reprojected = reprojected.reshape(-1, 2)

        for pt1, pt2 in zip(detected, reprojected):
            cv2.circle(img_display, tuple(pt1.astype(int)), 4, (0, 0, 255), -1)  # Red: detected
            cv2.circle(img_display, tuple(pt2.astype(int)), 2, (0, 255, 0), -1)  # Green: reprojected
            cv2.line(img_display, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (255, 0, 0), 1)

        cv2.imshow(f"Reprojection Error {i}", img_display)
        cv2.waitKey(0)

    cv2.destroyAllWindows()


# === Path configuration ===
base_dir = os.path.dirname(os.path.abspath(__file__))
calib_dir = os.path.join(base_dir, "calib_pics")
param_dir = os.path.join(base_dir, "param")
os.makedirs(calib_dir, exist_ok=True)
os.makedirs(param_dir, exist_ok=True)

# === Desired capture resolution (forced) ===
DESIRED_W = 320
DESIRED_H = 240

if __name__ == '__main__':
    # === Checkerboard specification ===
    CHECKERBOARD = (10, 7)  # (columns, rows) of inner corners
    square_size = 0.022     # square size in meters

    # Create 3D object points (same for all images)
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= square_size

    # Sub-pixel corner refinement criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # === 相机选择 ===
    # 如果是 Logitech：可以传 width/height 给类（若实现了），否则我们会在 open 后强制设置
    # cam = get_camera("LogitechCamera", source=1, width=DESIRED_W, height=DESIRED_H, fps=30)

    # ESP32 示例：使用 snapshot/stream url（如果类接受 width/height，可传）
    cam = get_camera("ESP32Camera", url="http://172.20.10.2", mode="stream")


    print("📷 Press SPACE to capture image (saved as-is)")
    print("🔴 Press Q to finish capture and start calibration\n")

    captured = 0
    with cam:
        # 尝试在打开后强制设置分辨率（兼容不同相机实现）
        try:
            # 优先使用相机类提供的接口（LogitechCamera 有 set_property）
            cam.set_property(cv2.CAP_PROP_FRAME_WIDTH, DESIRED_W)
            cam.set_property(cv2.CAP_PROP_FRAME_HEIGHT, DESIRED_H)
            print(f"[INFO] Requested camera properties via cam.set_property -> {DESIRED_W}x{DESIRED_H}")
        except Exception:
            # 回退：直接操作内部的 VideoCapture，如果存在的话
            try:
                if hasattr(cam, "_capture") and cam._capture is not None:
                    cam._capture.set(cv2.CAP_PROP_FRAME_WIDTH, DESIRED_W)
                    cam._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, DESIRED_H)
                    print(f"[INFO] Requested camera properties via cam._capture -> {DESIRED_W}x{DESIRED_H}")
            except Exception:
                print("[WARN] Could not force camera resolution; camera class may not support property setting.")

        # 进入主采集循环
        first_frame_checked = False
        while True:
            frame = cam.read()

            # 在第一次读到帧后检查实际分辨率
            if not first_frame_checked:
                if frame is None:
                    print("[WARN] First frame is None")
                else:
                    h, w = frame.shape[:2]
                    print(f"[INFO] First frame actual size: {w}x{h}")
                    if (w, h) != (DESIRED_W, DESIRED_H):
                        print(f"[WARN] Actual frame size {w}x{h} != desired {DESIRED_W}x{DESIRED_H}. "
                              "If using ESP32 ensure firmware frame_size is set to VGA(640x480) and jpeg_quality increased.")
                first_frame_checked = True

            cv2.imshow("Live Feed", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # Save raw image on SPACE
                filename = os.path.join(calib_dir, f"calib_{captured:02d}.png")
                cv2.imwrite(filename, frame)
                print(f"📸 Saved: {filename}")
                captured += 1

            elif key == ord('q'):  # Quit and start processing
                break

    cv2.destroyAllWindows()
    print(f"\n📂 Total images captured: {captured}")
    print("🔎 Starting checkerboard detection and calibration...")

    # === Post-capture processing ===
    image_files = sorted([os.path.join(calib_dir, f) for f in os.listdir(calib_dir) if f.endswith(".png")])
    objpoints = []
    imgpoints = []
    gray_shape = (DESIRED_W, DESIRED_H)  # default fallback

    for fname in image_files:
        img = cv2.imread(fname)
        if img is None:
            print(f"[WARN] Failed to read {fname}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Update gray_shape from actual image (use last valid)
        gray_shape = gray.shape[::-1]

        # Find corners in the image
        ret, corners = cv2.findChessboardCornersSB(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp.copy())
            imgpoints.append(corners2)
            print(f"✔️ Corners found: {fname}")
        else:
            print(f"❌ Corners NOT found: {fname}")

    print(f"\n📊 Usable images: {len(objpoints)} / {len(image_files)}")
    if len(objpoints) < 5:
        print("⚠️ Not enough valid images for calibration. Minimum 5 needed.")
        exit()

    # === Camera calibration ===
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

    # === Reprojection error computation ===
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    print(f"\n✅ Calibration complete. Reprojection error: {total_error / len(objpoints):.4f}")

    # === Save calibration parameters ===
    np.savetxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), mtx, delimiter=',')
    np.savetxt(os.path.join(param_dir, "distortion_coeffs.csv"), dist, delimiter=',')
    np.savetxt(os.path.join(param_dir, "image_size.txt"), np.array([gray_shape]), fmt='%d')

    print(f"\n📁 Calibration parameters saved to: {param_dir}")
    print(f"📷 Intrinsic matrix:\n{mtx}")
