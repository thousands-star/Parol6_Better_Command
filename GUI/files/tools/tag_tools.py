# tools/tag_utils.py
from __future__ import annotations
from types import SimpleNamespace
from typing import Any, Iterable, List, Optional, Sequence, Tuple
import re
import math

try:
    import numpy as np
except Exception:
    np = None

# ========= Pose-from-two-tags thread =========
def _tag_object_corners(center_xy, tag_size):
    """物体坐标系下单块 tag 的四角 (TL, TR, BR, BL)，Z=0"""
    cx, cy = center_xy
    s = tag_size * 0.5
    TL = (cx - s, cy - s, 0.0)
    TR = (cx + s, cy - s, 0.0)
    BR = (cx + s, cy + s, 0.0)
    BL = (cx - s, cy + s, 0.0)
    return np.array([TL, TR, BR, BL], dtype=np.float32)

def _R_to_euler_xyz(R, degrees=True):
    # 数值稳健：正交化
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    sy = -R[2, 0]
    ry = np.arcsin(np.clip(sy, -1.0, 1.0))
    if abs(np.cos(ry)) > 1e-6:
        rx = np.arctan2(R[2, 1], R[2, 2])
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        rz = 0.0
    if degrees:
        rx, ry, rz = np.degrees([rx, ry, rz])
    return float(rx), float(ry), float(rz)

# --- basic id extraction ---
def get_id(item: Any) -> Optional[int]:
    try:
        if hasattr(item, "tag_id"):
            return int(getattr(item, "tag_id"))
    except Exception:
        pass
    try:
        if isinstance(item, dict) and "tag_id" in item:
            return int(item["tag_id"])
    except Exception:
        pass
    if isinstance(item, (list, tuple)) and len(item) >= 1:
        try:
            return int(item[0])
        except Exception:
            pass
    if isinstance(item, str):
        m = re.search(r"Tag\s*(\d+)", item)
        if m:
            try:
                return int(m.group(1))
            except Exception:
                pass
    return None

# --- parse publisher string like "Tag 6: center=(u,v) corners=[(...)]" ---
def parse_publisher_string(s: str) -> List[SimpleNamespace]:
    out = []
    if not isinstance(s, str):
        return out
    pat = r"Tag\s*(\d+):\s*center=\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)\s*corners=\[([^\]]+)\]"
    for m in re.finditer(pat, s):
        try:
            tid = int(m.group(1))
            cu = float(m.group(2)); cv = float(m.group(3))
            corners_str = m.group(4)
            pairs = re.findall(r"\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)", corners_str)
            corners = [(float(u), float(v)) for (u, v) in pairs][:4]
            out.append(SimpleNamespace(tag_id=tid, center=(cu, cv), corners=corners))
        except Exception:
            continue
    return out

# --- normalize a variety of corner representations into 4 (u,v) pairs ---
def _norm4(pairs) -> Optional[List[Tuple[float, float]]]:
    try:
        # case: list of pairs
        if isinstance(pairs, (list, tuple)) and len(pairs) >= 4 and all(isinstance(p, (list,tuple)) and len(p)>=2 for p in pairs[:4]):
            return [(float(p[0]), float(p[1])) for p in pairs[:4]]
    except Exception:
        pass
    # try to flatten numbers
    nums = []
    try:
        for x in pairs:
            if isinstance(x, (list, tuple)):
                for y in x:
                    nums.append(float(y))
            else:
                nums.append(float(x))
        if len(nums) >= 8:
            return [(nums[0], nums[1]), (nums[2], nums[3]), (nums[4], nums[5]), (nums[6], nums[7])]
    except Exception:
        pass
    return None

def get_corners(item: Any) -> Optional[List[Tuple[float, float]]]:
    # attr
    try:
        if hasattr(item, "corners") and getattr(item, "corners") is not None:
            pts = _norm4(getattr(item, "corners"))
            if pts: return pts
    except Exception:
        pass
    # dict
    try:
        if isinstance(item, dict) and "corners" in item and item["corners"] is not None:
            pts = _norm4(item["corners"])
            if pts: return pts
    except Exception:
        pass
    # tuple/list forms
    if isinstance(item, (list, tuple)):
        if len(item) >= 4:
            pts = _norm4(item[3])
            if pts: return pts
        # flattened 8+ numbers
        if len(item) >= 8:
            try:
                flat = [float(x) for x in item[:8]]
                return [(flat[0],flat[1]),(flat[2],flat[3]),(flat[4],flat[5]),(flat[6],flat[7])]
            except Exception:
                pass
    # string parse
    if isinstance(item, str):
        parsed = parse_publisher_string(item)
        if parsed:
            return parsed[0].corners if getattr(parsed[0], "corners", None) else None
    return None

# --- centroid & polygon area ---
def compute_centroid(corners: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    xs = [p[0] for p in corners]; ys = [p[1] for p in corners]
    return (sum(xs)/len(xs), sum(ys)/len(ys))

def polygon_area(corners: Sequence[Tuple[float, float]]) -> float:
    if not corners or len(corners) < 3: return 0.0
    a = 0.0
    n = len(corners)
    for i in range(n):
        x0,y0 = corners[i]; x1,y1 = corners[(i+1)%n]
        a += x0*y1 - x1*y0
    return abs(a)*0.5

# --- pose helpers (try common field names) ---
def get_pose_t(item: Any) -> Optional[Tuple[float,float,float]]:
    names = ("pose_t","pose_tvec","pose_t_vec","tvec","pose_t_xyz")
    try:
        for n in names:
            if hasattr(item, n):
                v = getattr(item, n)
                return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:
        pass
    if isinstance(item, dict):
        for n in names:
            if n in item and item[n] is not None:
                try:
                    v = item[n]; return (float(v[0]), float(v[1]), float(v[2]))
                except Exception:
                    pass
    return None

def get_pose_rvec(item: Any) -> Optional[Tuple[float,float,float]]:
    names = ("pose_rvec","rvec","rotvec","pose_r")
    try:
        for n in names:
            if hasattr(item, n):
                v = getattr(item, n); return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:
        pass
    if isinstance(item, dict):
        for n in names:
            if n in item and item[n] is not None:
                try:
                    v = item[n]; return (float(v[0]), float(v[1]), float(v[2]))
                except Exception:
                    pass
    return None

# --- normalize into SimpleNamespace (convenient) ---
def normalize_item(item: Any) -> Optional[SimpleNamespace]:
    if item is None: return None
    if isinstance(item, str):
        parsed = parse_publisher_string(item)
        if parsed:
            item = parsed[0]
    tid = get_id(item)
    center = None
    try:
        if hasattr(item, "center") and getattr(item, "center") is not None:
            c = getattr(item, "center"); center = (float(c[0]), float(c[1]))
    except Exception:
        center = None
    if center is None and isinstance(item, (list,tuple)) and len(item) >= 3:
        try:
            center = (float(item[1]), float(item[2]))
        except Exception:
            center = None
    corners = get_corners(item)
    area = polygon_area(corners) if corners else None
    centroid = compute_centroid(corners) if corners else (center if center is not None else None)
    pose_t = get_pose_t(item)
    pose_rvec = get_pose_rvec(item)
    return SimpleNamespace(tag_id=tid, center=center, corners=corners, area=area, centroid=centroid, pose_t=pose_t, pose_rvec=pose_rvec)

# --- high level builder for IBVS (obs_uvs, Z_vals) ---
def build_obs_from_items(items: Iterable[Any], *, use_pose_depth: bool=True, depth_default: float=0.5) -> (List[Tuple[float,float]], List[float]):
    obs_uvs: List[Tuple[float,float]] = []
    Z_vals: List[float] = []
    for it in items:
        ns = normalize_item(it)
        if ns is None: continue
        corners = getattr(ns, "corners", None)
        if not corners: continue
        z = None
        if use_pose_depth:
            try:
                if getattr(ns, "pose_t", None) is not None:
                    z = float(ns.pose_t[2])
            except Exception:
                z = None
        if z is None:
            z = float(depth_default)
        for c in corners:
            obs_uvs.append((float(c[0]), float(c[1])))
            Z_vals.append(z)
    return obs_uvs, Z_vals

# -------------------------
# Small demo when run as script
# -------------------------
if __name__ == "__main__":
    # small sanity checks
    examples = [
        (6, 200.0, 150.0, [(100,100),(200,100),(200,200),(100,200)]),
        {"tag_id":9, "center":(120,123), "corners":[(10,10),(20,10),(20,20),(10,20)], "pose_t":[0.0,0.0,0.45]},
        "Tag 6: center=(201,163) corners=[(201,163),(200,135),(171,137),(171,164)]",
        (6,200,150,10,10,20,10,20,20,10,20),
        [10,11,12,13,14,15,16,17]
    ]
    for ex in examples:
        ns = normalize_item(ex)
        print("EX:", ex)
        print("  -> id:", get_id(ex))
        print("  -> corners:", get_corners(ex))
        print("  -> normalized:", ns)
        print("  -> centroid:", ns.centroid if ns else None)
        print()

