import os
import cv2
import time
import numpy as np
from CameraBase import LogitechCamera  # Make sure this class is implemented
import matplotlib.pyplot as plt

def visualize_reprojection_errors(images, objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    """
    Visualize reprojection errors by comparing detected vs. reprojected points.
    Red: detected image points
    Green: reprojected points from 3D object points
    Blue lines: visual error vectors
    """
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

    cam = LogitechCamera(source=1, width=640, height=480, fps=30)

    print("📷 Press SPACE to capture image (saved as-is)")
    print("🔴 Press Q to finish capture and start calibration\n")

    captured = 0
    with cam:
        while True:
            frame = cam.read()
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

    for fname in image_files:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

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
    gray_shape = gray.shape[::-1]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

    # === Reprojection error computation ===
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    print(f"\n✅ Calibration complete. Reprojection error: {total_error / len(objpoints):.4f}")

    # === Optional: visualize reprojection errors ===
    # visualize_reprojection_errors(image_files, objpoints, imgpoints, rvecs, tvecs, mtx, dist)

    # === Save calibration parameters ===
    np.savetxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), mtx, delimiter=',')
    np.savetxt(os.path.join(param_dir, "distortion_coeffs.csv"), dist, delimiter=',')
    np.savetxt(os.path.join(param_dir, "image_size.txt"), np.array([gray_shape]), fmt='%d')

    print(f"\n📁 Calibration parameters saved to: {param_dir}")
    print(f"📷 Intrinsic matrix:\n{mtx}")
