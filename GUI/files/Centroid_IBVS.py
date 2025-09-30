# tools/centroid_ibvs_reactor.py
import os
import math
import queue
import numpy as np
from typing import Optional, Iterable, Tuple, List
from multiprocessing import Array

from Reactor import Reactor, INTERVAL_S
from tools.shared_struct import RobotInputData
from action import SingleCartesianJogAction, DummyAction
import tools.PAROL6_ROBOT as PAROL6_ROBOT


class CentroidIBVSReactor(Reactor):
    def __init__(
        self,
        detected_tags_queue: queue.Queue,
        jog_control: Array,
        shared_string: Optional[Array],
        desired_uv: Optional[Tuple[float, float]] = None,
        fx: Optional[float] = None,
        fy: Optional[float] = None,
        depth_m: float = 0.5,
        lam: float = 1.0,
        deadband_px: float = 4.0,
        max_step_m: Optional[float] = None,
        invert_x: bool = False,
        invert_y: bool = True,
        use_target_ids: Optional[List[int]] = None,
        frame: str = "TRF",
        speed_pct_default: float = 30.0,
        interval_s: float = INTERVAL_S,
    ):
        """
        If a valid camera_intrinsic_matrix.csv exists at tools/Camera/param, this ctor will load it
        and override fx,fy and desired_uv (cx,cy) automatically.
        """
        super().__init__()
        self.tags_q = detected_tags_queue
        self.jog_control = jog_control
        self.shared_string = shared_string

        # defaults (may be overridden by K)
        self.fx = fx
        self.fy = fy
        self.cx = None
        self.cy = None

        # try to load camera intrinsics from repo param folder (same path Vision.py uses)
        try:
            param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
            K_path = os.path.join(param_dir, "camera_intrinsic_matrix.csv")
            if os.path.exists(K_path):
                K = np.loadtxt(K_path, delimiter=',').astype(float)
                # K assumed 3x3: [[fx,  0, cx],
                #                 [ 0, fy, cy],
                #                 [ 0,  0,  1]]
                self.fx = float(K[0, 0])
                self.fy = float(K[1, 1])
                self.cx = float(K[0, 2])
                self.cy = float(K[1, 2])
                if self.shared_string:
                    try:
                        self.shared_string.value = b"CentroidIBVS: loaded K from param"
                    except Exception:
                        pass
            else:
                # no file present, fall through to ctor fx/fy
                pass
        except Exception as exc:
            print("[CentroidIBVS] Warning: cannot load camera intrinsics:", exc)
            # fallback to provided fx/fy below

        # If desired_uv not provided, use principal point cx,cy if available else fallback to center (640x480)
        if desired_uv is not None:
            self.desired_uv = (float(desired_uv[0]), float(desired_uv[1]))
        elif (self.cx is not None) and (self.cy is not None):
            self.desired_uv = (self.cx, self.cy)
        else:
            # fallback default image center if nothing known
            self.desired_uv = (320.0, 240.0)

        # if fx/fy still None, set conservative defaults
        if self.fx is None: self.fx = 800.0
        if self.fy is None: self.fy = 800.0

        self.depth = float(depth_m)
        self.lam = float(lam)
        self.deadband_px = float(deadband_px)
        self.max_step_m = max_step_m if (max_step_m is not None) else PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG * interval_s
        self.invert_x = bool(invert_x)
        self.invert_y = bool(invert_y)
        self.target_ids = None if use_target_ids is None else set(use_target_ids)
        self.frame = frame
        self.speed_pct_default = float(speed_pct_default)
        self.interval_s = float(interval_s)

        self.last_centroid = None
        self.last_err_px = (0.0, 0.0)

    def reset(self, robot_data: RobotInputData):
        pass
    
    def _centroid_from_taglist(self, taglist) -> Optional[Tuple[float,float]]:
        if not taglist:
            return None
        pts = []
        for t in taglist:
            if isinstance(t, dict):
                tid = t.get("id", None) or t.get("tag_id", None)
                c = t.get("center", None)
            else:
                tid = getattr(t, "tag_id", None) or getattr(t, "id", None)
                c = getattr(t, "center", None)
            if (self.target_ids is not None) and (tid not in self.target_ids):
                continue
            if c is None:
                continue
            try:
                u = float(c[0]); v = float(c[1])
            except Exception:
                continue
            pts.append((u, v))
        if not pts:
            return None
        u_avg = sum(p[0] for p in pts) / len(pts)
        v_avg = sum(p[1] for p in pts) / len(pts)
        return (u_avg, v_avg)

    def plan(self, robot_data: RobotInputData):
        try:
            taglist = self.tags_q.get_nowait()
        except queue.Empty:
            return DummyAction(robot_data.position)

        centroid = self._centroid_from_taglist(taglist)
        if centroid is None:
            if self.shared_string:
                try:
                    self.shared_string.value = b"CentroidIBVS: no tags"
                except Exception:
                    pass
            return DummyAction(robot_data.position)

        u_cur, v_cur = centroid
        u_des, v_des = self.desired_uv
        u_err = (u_des - u_cur)
        v_err = (v_des - v_cur)

        self.last_centroid = centroid
        self.last_err_px = (u_err, v_err)

        if (abs(u_err) <= self.deadband_px) and (abs(v_err) <= self.deadband_px):
            if self.shared_string:
                try:
                    self.shared_string.value = b"CentroidIBVS: in deadband"
                except Exception:
                    pass
            return DummyAction(robot_data.position)

        dx_cam = (u_err / max(1e-6, self.fx)) * self.depth
        dy_cam = (v_err / max(1e-6, self.fy)) * self.depth

        if self.invert_x: dx_cam = -dx_cam
        if self.invert_y: dy_cam = -dy_cam

        dx = float(self.lam * dx_cam)
        dy = float(self.lam * dy_cam)
        dz = 0.0

        mag = math.hypot(dx, dy)
        if mag > 0 and mag > self.max_step_m:
            scale = self.max_step_m / mag
            dx *= scale; dy *= scale

        if self.shared_string:
            try:
                txt = f"CentroidIBVS err_px=({u_err:.1f},{v_err:.1f}) -> d_m=({dx:.4f},{dy:.4f})"
                self.shared_string.value = txt.encode()[:120]
            except Exception:
                pass

        speed_pct = self.speed_pct_default if (self.jog_control is None) else float(self.jog_control[0])
        return SingleCartesianJogAction(
            dx=dx, dy=dy, dz=dz,
            rx=0.0, ry=0.0, rz=0.0,
            frame=self.frame,
            speed_pct=speed_pct,
            interval_s=self.interval_s,
            shared_string=self.shared_string
        )

    def to_dict(self):
        return {
            "type": "CentroidIBVS",
            "desired_uv": self.desired_uv,
            "last_centroid": self.last_centroid,
            "last_err_px": {"u_err": self.last_err_px[0], "v_err": self.last_err_px[1]},
            "depth_m": self.depth,
            "lam": self.lam,
            "deadband_px": self.deadband_px,
            "max_step_m": self.max_step_m,
            "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy
        }
