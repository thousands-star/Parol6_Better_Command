# pbvs_coarse_reactor.py
from __future__ import annotations
import time, queue
import numpy as np
import cv2
from dataclasses import dataclass
from spatialmath import SE3
import os

from Reactor import Reactor
import tools.PAROL6_ROBOT as PAROL6_ROBOT
from action import Move2JointsAction, SingleCartesianJogAction, DummyAction

@dataclass
class PBVSParams:
    tag_size_m: float = 0.048
    standoff: float = 0.08
    push_dist: float = 0.06
    step_trf: float = 0.01
    push_sign: int = -1           # +1: 沿工具 +Z；-1: 沿工具 -Z
    yaw_only: bool = True
    # 质量门限
    min_margin: float = 40.0
    max_reproj_px: float = 2.0
    max_view_angle_deg: float = 75.0
    fresh_ms: int = 300
    # 时间滤波
    alpha_t: float = 0.25
    alpha_r: float = 0.20

class PBVSCoarseReactor(Reactor):
    """
    WAIT_TAG → ROT_ALIGN(姿态) → XY_SLIDE(位置) → PUSH_TRF(沿工具Z直推) → DONE
    - 只返回 Action（Move2JointsAction / SingleCartesianJogAction / DummyAction）
    - q 从 robot_data.position 取
    - camera=工具轴：T_tcp_cam 默认单位阵
    - K 自动从项目里取（Vision.K / camera.K / tools.camera.K）
    """
    def __init__(self, tags_q, T_tcp_cam: SE3 | None = None, params: PBVSParams = PBVSParams()):
        super().__init__()
        self.tags_q = tags_q
        self.T_tcp_cam = T_tcp_cam if T_tcp_cam is not None else SE3()  # camera=工具轴
        self.p = params

        self.state = "WAIT_TAG"
        self._pose_ema: SE3 | None = None
        self._last_pose_ts = 0.0
        self.T_w_goal: SE3 | None = None
        self._push_total = 0
        self._push_done = 0

        self._K = self._discover_K()
        self._dist = self._discover_dist()

    # -------- 核心：仅返回 Action --------
    def plan(self, robot_data):
        q_now = self._q(robot_data)
        T_w_tcp = PAROL6_ROBOT.robot.fkine(q_now)
        now = time.time()

        if self.state == "WAIT_TAG":
            T_cam_tag = self._fused_pose_from_queue()
            fresh = (T_cam_tag is not None) and ((now - self._last_pose_ts)*1000.0 <= self.p.fresh_ms)
            if not fresh:
                return DummyAction(robot_data.position)

            # 世界系 tag → 粗对准目标
            T_w_tag = T_w_tcp * self.T_tcp_cam * T_cam_tag
            z_tool_des = -T_w_tag.R[:,2]
            R_align = self._R_from_z(z_tool_des, self.p.yaw_only)
            T_tag_tool = SE3.Rt(R_align, [0,0,self.p.standoff])
            self.T_w_goal = T_w_tag * T_tag_tool

            # 只旋姿
            self.state = "ROT_ALIGN"
            T_rot_goal = SE3.Rt(self.T_w_goal.R, T_w_tcp.t)
            q1 = self._ik(T_rot_goal, q_now)
            return Move2JointsAction(q1, speed_pct=40.0)

        if self.state == "ROT_ALIGN":
            # XY 平移
            self.state = "XY_SLIDE"
            T_xy_goal = SE3.Rt(self.T_w_goal.R, [self.T_w_goal.t[0], self.T_w_goal.t[1], T_w_tcp.t[2]])
            q2 = self._ik(T_xy_goal, q_now)
            return Move2JointsAction(q2, speed_pct=35.0)

        if self.state == "XY_SLIDE":
            # TRF 直推（分步）
            self.state = "PUSH_TRF"
            self._push_total = max(1, int(abs(self.p.push_dist)/self.p.step_trf))
            self._push_done = 0
            return self._one_push_step(robot_data)

        if self.state == "PUSH_TRF":
            if self._push_done < self._push_total:
                return self._one_push_step(robot_data)
            self.state = "DONE"
            return DummyAction(robot_data.position)

        if self.state == "DONE":
            return DummyAction(robot_data.position)

        return DummyAction(robot_data.position)

    # -------- 一步直推（返回 SingleCartesianJogAction）--------
    def _one_push_step(self, robot_data):
        dz = self.p.push_sign * self.p.step_trf
        self._push_done += 1
        return SingleCartesianJogAction(
            dx=0.0, dy=0.0, dz=float(dz),
            rx=0.0, ry=0.0, rz=0.0,
            frame="TRF",
            speed_pct=30.0,
            interval_s=0.01,
            shared_string=None
        )

    # -------- Vision 融合（从 tags_q 拿原始检测，自行稳 pose）--------
    def _fused_pose_from_queue(self) -> SE3 | None:
        dets = None
        while True:
            try:
                dets = self.tags_q.get_nowait()
            except queue.Empty:
                break
        if dets is None or len(dets) == 0:
            return self._pose_ema

        obj_list, img_list = [], []
        for d in dets:
            c = self._corners(d); m = self._margin(d)
            if c is None: continue
            if m is None: m = 0.0
            if m < self.p.min_margin: continue
            R0 = self._R_if_any(d)
            if R0 is not None and self._ang_deg(R0) > self.p.max_view_angle_deg:
                continue
            obj_list.append(self._obj_corners(self.p.tag_size_m))
            img_list.append(c.astype(np.float32))

        if len(img_list) == 0:
            return self._pose_ema

        obj = np.vstack(obj_list).astype(np.float32)
        img = np.vstack(img_list).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(obj, img, self._K, self._dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return self._pose_ema

        proj, _ = cv2.projectPoints(obj, rvec, tvec, self._K, self._dist)
        reproj = float(np.linalg.norm(proj.reshape(-1,2) - img, axis=1).mean())
        if reproj > self.p.max_reproj_px:
            return self._pose_ema

        R, t = self._Rt(rvec, tvec)
        if self._ang_deg(R) > self.p.max_view_angle_deg:
            return self._pose_ema

        T_new = SE3.Rt(R, t)
        self._last_pose_ts = time.time()
        if self._pose_ema is None:
            self._pose_ema = T_new
        else:
            # 平移 EMA
            t_blend = (1.0 - self.p.alpha_t)*self._pose_ema.t + self.p.alpha_t*T_new.t
            # 旋转 slerp（小角度近似）
            R_blend = self._slerp(self._pose_ema.R, T_new.R, self.p.alpha_r)
            self._pose_ema = SE3.Rt(R_blend, t_blend)
        return self._pose_ema

    # -------- K / dist 自动发现 --------
    @staticmethod
    def _discover_K() -> np.ndarray:
        # 优先 Vision.K；否则 camera.K / tools.camera.K
        # 相机内参
        K = None
        try:
            param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
            K = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"),
                                delimiter=',').astype(float)
            return K
        except Exception as exc:
            print("[IBVS] Warning: cannot load camera intrinsics:", exc)
            pass

        raise RuntimeError("Camera intrinsics K 未找到：请在 Vision.K 或 camera.K 提供 3x3 矩阵")

    @staticmethod
    def _discover_dist():
        # 可选：如果项目里有 dist / D / dist_coeffs 就取；否则 None
        for (modname, attr) in (("Vision","dist"), ("Vision","D"), ("camera","dist"), ("tools.camera","dist")):
            try:
                mod = __import__(modname, fromlist=[attr])
                if hasattr(mod, attr):
                    d = np.asarray(getattr(mod, attr), dtype=np.float64).reshape(-1,1)
                    return d
            except Exception:
                continue
        return None

    # -------- 机器人/几何工具 --------
    @staticmethod
    def _q(robot_data) -> np.ndarray:
        arr = np.asarray(robot_data.position, dtype=float)
        return arr[:6] if arr.size >= 6 else arr

    def _ik(self, T_goal: SE3, q0: np.ndarray) -> np.ndarray:
        we_pos = (1,1,1)
        we_rot = (20,20,35) if self.p.yaw_only else (20,20,20)
        we = np.diag([*we_pos, *we_rot])
        sol = PAROL6_ROBOT.robot.ikine_LMS(T_goal, q0=q0, ilimit=30, we=we, mask=[1,1,1,1,1,1])
        if getattr(sol, "success", True) is False:
            sol = PAROL6_ROBOT.robot.ikine_LMS(T_goal, q0=q0, ilimit=60)
        return sol.q

    @staticmethod
    def _obj_corners(size_m: float) -> np.ndarray:
        s = size_m/2.0
        return np.array([[-s,-s,0],[+s,-s,0],[+s,+s,0],[-s,+s,0]], dtype=np.float32)

    @staticmethod
    def _Rt(rvec, tvec):
        R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
        t = np.asarray(tvec, dtype=np.float64).reshape(3)
        return R, t

    @staticmethod
    def _R_from_z(z_tool_des: np.ndarray, yaw_only: bool) -> np.ndarray:
        z = z_tool_des / (np.linalg.norm(z_tool_des) + 1e-9)
        if yaw_only:
            from spatialmath import SE3 as _SE3
            return _SE3.Rz(float(np.arctan2(z[1], z[0]))).R
        tmp = np.array([0,0,1.0]); x = np.cross(tmp, z)
        if np.linalg.norm(x) < 1e-6:
            tmp = np.array([0,1,0]); x = np.cross(tmp, z)
        x = x/(np.linalg.norm(x)+1e-9); y = np.cross(z, x)
        return np.column_stack((x,y,z))

    @staticmethod
    def _ang_deg(R: np.ndarray) -> float:
        z = R[:,2]
        cosang = np.clip(z[2]/(np.linalg.norm(z)+1e-9), -1.0, 1.0)
        return float(np.degrees(np.arccos(cosang)))

    @staticmethod
    def _slerp(R0: np.ndarray, R1: np.ndarray, a: float) -> np.ndarray:
        dR = R0.T @ R1
        w_hat = 0.5*(dR - dR.T)
        w = np.array([w_hat[2,1], w_hat[0,2], w_hat[1,0]])
        th = np.linalg.norm(w)
        if th < 1e-9: return R0
        k = w/th; ang = a*th
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
        return R0 @ (np.eye(3) + np.sin(ang)*K + (1-np.cos(ang))*(K@K))

    @staticmethod
    def _corners(det) -> np.ndarray | None:
        c = None
        if hasattr(det, "corners"): c = np.asarray(det.corners, dtype=np.float32)
        elif isinstance(det, dict) and "corners" in det: c = np.asarray(det["corners"], np.float32)
        if c is None or c.shape != (4,2): return None
        return c

    @staticmethod
    def _margin(det) -> float | None:
        if hasattr(det, "decision_margin"): return float(det.decision_margin)
        if isinstance(det, dict) and "decision_margin" in det: return float(det["decision_margin"])
        return 0.0

    @staticmethod
    def _R_if_any(det) -> np.ndarray | None:
        if isinstance(det, dict) and "R" in det:
            return np.asarray(det["R"], dtype=np.float64).reshape(3,3)
        if hasattr(det, "pose_R"):
            return np.asarray(det.pose_R, dtype=np.float64).reshape(3,3)
        return None
