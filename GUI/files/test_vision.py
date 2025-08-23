import cv2, numpy as np, os
from pathlib import Path
from pupil_apriltags import Detector

IMG_PATH = r"C:\Users\Public\fyp\PAROL-commander-software-main\esp_simple_out\out_fail.jpg"   # <-- 改成你的快照文件
K_PATH = r"C:\Users\Public\fyp\PAROL-commander-software-main\GUI\files\tools\Camera\param\camera_intrinsic_matrix.csv"  # 若有内参
OUT = "center_check.jpg"

img = cv2.imread(IMG_PATH)
if img is None:
    raise SystemExit("Can't read image: " + IMG_PATH)
h, w = img.shape[:2]

# 1) image center vs intrinsic principal point (if available)
if Path(K_PATH).exists():
    K = np.loadtxt(K_PATH, delimiter=',')
    if K.size == 9:
        K = K.reshape(3,3)
    cx, cy = float(K[0,2]), float(K[1,2])
    print("Principal point from K: cx,cy =", cx, cy)
else:
    K = None
    cx, cy = None, None
    print("No intrinsic file found at", K_PATH)

img2 = img.copy()
# draw image geometric center
cv2.drawMarker(img2, (w//2, h//2), (255,0,0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
cv2.putText(img2, "ImageCenter", (w//2+8, h//2+8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

if cx is not None:
    cv2.drawMarker(img2, (int(round(cx)), int(round(cy))), (0,255,0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)
    cv2.putText(img2, f"Principal({cx:.1f},{cy:.1f})", (int(cx)+8, int(cy)+8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    print("Image center (w/2,h/2):", (w/2, h/2))
    print("Offset (principal - geometric) px: dx=", cx-w/2, " dy=", cy-h/2)

# 2) detect apriltags centers (you said tagStandard41h12)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
det = Detector(families='tagStandard41h12', nthreads=2)
tags = det.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)
print("Detected tags:", len(tags))
centers = []
for t in tags:
    u,v = float(t.center[0]), float(t.center[1])
    centers.append((u,v))
    cv2.circle(img2, (int(u), int(v)), 6, (0,255,255), -1)
    cv2.putText(img2, f"id{int(t.tag_id)}", (int(u)+6,int(v)-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

if centers:
    centers_arr = np.array(centers)
    mean = centers_arr.mean(axis=0)
    std = centers_arr.std(axis=0)
    print("Tags center mean (u,v):", mean, " std:", std)
    # distance from image center
    deltas = centers_arr - np.array([w/2, h/2])
    dists = np.linalg.norm(deltas, axis=1)
    print("Mean distance of tags to image center (px):", dists.mean(), " max:", dists.max())

# 3) RGB channel alignment check (edge overlays)
b,g,r = cv2.split(img)
edges_b = cv2.Canny(b, 50, 150)
edges_g = cv2.Canny(g, 50, 150)
edges_r = cv2.Canny(r, 50, 150)

# create false-color overlay: R channel edges in red, G in green, B in blue
overlay = np.zeros_like(img)
overlay[:,:,0] = edges_b
overlay[:,:,1] = edges_g
overlay[:,:,2] = edges_r

# enlarge for visualization and combine
vis = cv2.addWeighted(img2, 0.7, overlay, 0.8, 0)

cv2.imwrite(OUT, vis)
print("Wrote visualization to", OUT)
