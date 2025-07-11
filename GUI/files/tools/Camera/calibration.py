import os
import cv2
import time
import numpy as np
from LogitechCamera import LogitechCamera  # 请确保你有这个类

base_dir = os.path.dirname(os.path.abspath(__file__))

calib_dir = os.path.join(base_dir, "calib_pics")
param_dir = os.path.join(base_dir, "param")

os.makedirs(calib_dir, exist_ok=True)
os.makedirs(param_dir, exist_ok=True)

if __name__ == '__main__':
    CHECKERBOARD = (7, 10)  # inner corners (rows, cols)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objpoints = []  # 3D points in real world
    imgpoints = []  # 2D points in image plane
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)

    os.makedirs("calib_pics", exist_ok=True)
    cam = LogitechCamera(source=1, width=640, height=480, fps=30)

    print("📷 Press SPACE to capture a checkerboard image.")
    print("🔴 Press ESC to finish and calibrate.\n")

    captured = 0
    with cam:
        while True:
            frame = cam.read()
            preview = frame  # ensure it's RGB if using LogitechCamera
            cv2.imshow("Live Feed", preview)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                break
            elif key == 32:  # SPACE
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCornersSB(
                    gray, CHECKERBOARD,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
                )
                if ret:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    objpoints.append(objp)
                    imgpoints.append(corners2)

                    img_with_corners = cv2.drawChessboardCorners(frame.copy(), CHECKERBOARD, corners2, ret)
                    filename = os.path.join(calib_dir, f"calib_{captured:02d}.png")
                    cv2.imwrite(filename, img_with_corners)
                    print(f"✔️  Saved: {filename}")
                    captured += 1
                else:
                    print("❌  Checkerboard not found. Try again.")

    cv2.destroyAllWindows()
    print(f"\n📸 Total usable calibration images: {len(objpoints)}")

    if len(objpoints) < 5:
        print("⚠️ Not enough valid images. Please capture at least 5 checkerboard images.")
        exit()

    gray_shape = gray.shape[::-1]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

    # Reprojection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    print(f"\n✅ Calibration complete. Reprojection error: {total_error / len(objpoints):.4f}")

    # Save calibration data
    param_dir = os.path.join(os.getcwd(), "param")
    os.makedirs(param_dir, exist_ok=True)
    # --- Save calibration results ---
    np.savetxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), mtx, delimiter=',')
    np.savetxt(os.path.join(param_dir, "distortion_coeffs.csv"), dist, delimiter=',')
    np.savetxt(os.path.join(param_dir, "image_size.txt"), np.array([gray_shape]), fmt='%d')

    print(f"\n📁 Calibration parameters saved to: {param_dir}")
    print(f"📷 Intrinsic matrix:\n{mtx}")
