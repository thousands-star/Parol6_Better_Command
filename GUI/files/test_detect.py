#!/usr/bin/env python3
# esp32_simple_detect.py — AprilTag (tagStandard41h12) snapshot detector

import os, time, requests, cv2, numpy as np
from pupil_apriltags import Detector

ESP_URL = "http://172.20.10.2/capture"   # ← 改成你的 /capture
OUTDIR  = "esp_simple_out"
os.makedirs(OUTDIR, exist_ok=True)

# 可选：某些固件会镜像；需要时改成 True 试试
FLIP_HORIZONTAL = True

det = Detector(families="tagStandard41h12", nthreads=2)

def fetch_snapshot(url, timeout=5.0):
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        arr = np.frombuffer(r.content, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img
    except Exception as e:
        print("snapshot error:", e)
        return None

def preprocess(gray):
    # 1) 增强对比（CLAHE）  2) 轻微锐化（unsharp）
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    g = clahe.apply(gray)
    g = cv2.addWeighted(g, 1.5, cv2.GaussianBlur(g, (0,0), 3), -0.5, 0)
    return g

def estimate_px_side(tag):
    c = np.array(tag.corners).reshape(-1,2)
    d = [np.linalg.norm(c[i]-c[(i+1)%4]) for i in range(4)]
    return float(np.mean(d))

if __name__ == "__main__":
    print("Starting detector @", ESP_URL)
    while True:
        img = fetch_snapshot(ESP_URL, timeout=5.0)
        if img is None:
            time.sleep(0.5); continue

        if FLIP_HORIZONTAL:
            img = cv2.flip(img, 1)

        h, w = img.shape[:2]
        print(f"frame: {w}x{h}", end="  ")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 清晰度评估（Laplacian 方差）
        lv = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        print(f"lapVar={lv:.1f}", end="  ")
        if lv < 80:
            print("[blur?]", end="  ")
        else:
            print("        ", end="  ")

        g = preprocess(gray)
        tags = det.detect(g, estimate_tag_pose=False, camera_params=None, tag_size=None)

        ts = int(time.time()*1000)
        if not tags:
            print("no tags")
            cv2.imwrite(os.path.join(OUTDIR, f"fail_{ts}.jpg"), img)
            cv2.imwrite(os.path.join(OUTDIR, f"fail_{ts}_preproc.jpg"), g)
        else:
            print(f"found {len(tags)} tag(s)")
            for t in tags:
                px = estimate_px_side(t)
                u, v = int(t.center[0]), int(t.center[1])
                # draw
                corners = np.array(t.corners, dtype=int).reshape(-1,2)
                for i in range(4):
                    cv2.line(img, tuple(corners[i]), tuple(corners[(i+1)%4]), (0,255,0), 2)
                cv2.circle(img, (u, v), 6, (0,255,0), -1)
                cv2.putText(img, f"id{int(t.tag_id)} px~{px:.1f}", (u+6, v-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                print(f"  id={int(t.tag_id)}  center=({u},{v})  px≈{px:.1f}")

            out = os.path.join(OUTDIR, f"ok_{ts}.jpg")
            cv2.imwrite(out, img)
            # 也存一份预处理图对照
            cv2.imwrite(os.path.join(OUTDIR, f"ok_{ts}_preproc.jpg"), g)

        time.sleep(0.25)
