from __future__ import annotations
import time
from abc import ABC, abstractmethod
from tools.shared_struct import RobotInputData, RobotOutputData, check_elements
import numpy as np
from typing import List, Dict, Any
from multiprocessing import Array
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
from action import SingleJointJogAction, SingleCartesianJogAction, HomeRobotAction, EnableRobotAction, DisableRobotAction, DummyAction, ClearErrorAction, Move2JointsAction
from tools.log_tools import nice_print_sections
from action import Action
import queue
from statistics import median           
from typing import List, Tuple, Optional
from tools.speed_tools import percent_to_joint_speed



INTERVAL_S = 0.01
Robot_mode = "Dummy"

def get_median_tag_offset(tags: List) -> Optional[Tuple[float, float, float]]:
    """
    Given a list of AprilTag detections (each with .pose_t as a 3-element array),
    return (dx, dy, dz) where each is the median of the corresponding pose_t entries.
    If no tags, returns None.
    """
    if not tags:
        return None

    # extract x, y, z from each tag.pose_t
    xs = [float(tag.pose_t[0]) for tag in tags]
    ys = [float(tag.pose_t[1]) for tag in tags]
    zs = [float(tag.pose_t[2]) for tag in tags]

    # median will handle both single‐element and multi‐element lists
    dx = median(xs)
    dy = median(ys)
    dz = median(zs)

    return dx, dy, dz


class Reactor(ABC):
    """
    Im thinking Reactor as a abstractize planning class that governs the high-level behavior of the arm
    For every reactor, the plan function has to be unique 
    this function would be called in the interval of 0.01s in the serial_sende
    """
    def __init__(self):
        self.prev_speed = [0,0,0,0,0,0]
        self.is_completed = False
        self.counter = 0

    def giveCommand(self, robot_data: RobotInputData) -> Action:
        """
        This would be called for every 10ms. It will parse in the cmd_data and modify the shared memory inside cmd_data
        Then return the cmd_data.pack() -> byte list
        """
        return self.plan(robot_data=robot_data)
    
    def is_completed(self) -> bool:
        return self.is_completed
    
    @abstractmethod
    def reset(self, robot_data: RobotInputData):
        pass
    
    @abstractmethod
    def plan(self, robot_data: RobotInputData) -> Action:
        pass
    
    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        """
        This method is to return a dictionary that logs important info.
        """
        pass


class GUIReactor(Reactor):
    def __init__(self, 
                shared_string:       Array,                  
                Joint_jog_buttons:   Array,             # multiprocessing.Array("i", [..])
                Cart_jog_buttons:    Array,             # multiprocessing.Array("i", [..])
                Jog_control:         Array,             # multiprocessing.Array("i", [..])
                Buttons:             Array,             # multiprocessing.Array("i", [..])
                 ):
        
        super().__init__()
        self.shared_string = shared_string
        self.Joint_jog_buttons = Joint_jog_buttons
        self.Cart_jog_buttons = Cart_jog_buttons
        self.Jog_control = Jog_control
        self.Buttons = Buttons

    def reset(self, robot_data: RobotInputData):
        pass


    def plan(self, robot_data: RobotInputData):
            # Check if any of jog buttons is pressed
            result_joint_jog = check_elements(list(self.Joint_jog_buttons))
            result_cart_jog = check_elements(list(self.Cart_jog_buttons))

            ######################################################
            ######################################################
            InOut_in = robot_data.inout
            Position_in = robot_data.position

            
            # JOINT JOG (regular speed control) 0x123 # -1 is value if nothing is pressed
            if result_joint_jog != -1 and self.Buttons[2] == 0 and InOut_in[4] == 1: 
                joint_id = result_joint_jog % 6
                direction = 1 if result_joint_jog < 6 else -1

                speed_val = percent_to_joint_speed(joint_id=joint_id, percent=self.Jog_control[0], direction=direction)
                return SingleJointJogAction(joint_id, speed_val,self.shared_string)

            ######################################################
            ######################################################
            # CART JOG (regular speed control but for multiple joints) 0x123 # -1 is value if nothing is pressed
            elif result_cart_jog != -1 and self.Buttons[2] == 0 and InOut_in[4] == 1: #

                Robot_mode = "Cartesian Jog"

                # Direction mapping (unit vector or rotation)
                jog_map = {
                    0: {"xyz": (-1, 0, 0)},
                    1: {"xyz": (1, 0, 0)},
                    2: {"xyz": (0, -1, 0)},
                    3: {"xyz": (0, 1, 0)},
                    4: {"xyz": (0, 0, 1)},
                    5: {"xyz": (0, 0, -1)},
                    6: {"rpy": (-1, 0, 0)},
                    7: {"rpy": (1, 0, 0)},
                    8: {"rpy": (0, 1, 0)},
                    9: {"rpy": (0, -1, 0)},
                    10: {"rpy": (0, 0, 1)},
                    11: {"rpy": (0, 0, -1)},
                }

                # Choose vector
                linear_v = np.interp(self.Jog_control[0], [0, 100], [
                    PAROL6_ROBOT.Cartesian_linear_velocity_min_JOG,
                    PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG,
                ])
                angular_v = np.interp(self.Jog_control[0], [0, 100], [
                    PAROL6_ROBOT.Cartesian_angular_velocity_min,
                    PAROL6_ROBOT.Cartesian_angular_velocity_max,
                ])

                delta_s = linear_v * INTERVAL_S
                delta_theta = angular_v * INTERVAL_S  # degrees

                mapping = jog_map[result_cart_jog]
                dx = dy = dz = rx = ry = rz = 0

                if "xyz" in mapping:
                    dx, dy, dz = [val * delta_s for val in mapping["xyz"]]
                if "rpy" in mapping:
                    rx, ry, rz = [val * delta_theta for val in mapping["rpy"]]

                # Execute jog (stateless action)
                return SingleCartesianJogAction(
                    dx=dx, dy=dy, dz=dz,
                    rx=rx, ry=ry, rz=rz,
                    frame="WRF" if self.Jog_control[2] == 1 else "TRF",
                    speed_pct=self.Jog_control[0],
                    interval_s=INTERVAL_S,
                    shared_string=self.shared_string
                )
                # Calculate every joint speed using var and q1

                # commanded position = robot position
                # if real send to real
                # if sim send to sim 
                # if both send to both 
                #print(result_joint_jog)

            elif self.Buttons[0] == 1:
                self.Buttons[0] = 0
                return HomeRobotAction(self.shared_string)

            elif self.Buttons[1] == 1:
                self.Buttons[1] = 0
                return EnableRobotAction(self.shared_string)

            elif self.Buttons[2] == 1 or InOut_in[4] == 0:
                self.Buttons[2] = 0
                self.Buttons[7] = 0
                return DisableRobotAction(self.shared_string)

            elif self.Buttons[3] == 1:
                self.Buttons[3] = 0
                return ClearErrorAction(self.shared_string)
            else: # If nothing else is done send dummy data 0x255
                return DummyAction(Position_in)

    def to_dict(self):
        gui = {
            "Joint jog":  list(self.Joint_jog_buttons),
            "Cart jog":   list(self.Cart_jog_buttons),
            "Home":       self.Buttons[0],
            "Enable":     self.Buttons[1],
            "Disable":    self.Buttons[2],
            "Clear err":  self.Buttons[3],
            "Real/Sim":   f"{self.Buttons[4]}/{self.Buttons[5]}",
            "Speed sl":   self.Jog_control[0],
            "WRF/TRF":    self.Jog_control[2],
            "Demo":       self.Buttons[6],
            "Execute":    self.Buttons[7],
            "Park":       self.Buttons[8],
            "Log msg":    self.shared_string.value.decode().strip(),
        }

        return gui
    
class FollowTagReactor(Reactor):
    def __init__(self, detected_tags: queue.Queue, jog_control: Array, shared_string: Array):
        self.tags_q = detected_tags
        self.jog_control = jog_control
        self.shared_string = shared_string
        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.tag = None
    
    def reset(self, robot_data: RobotInputData):
        pass
    
    def plan(self, robot_data: RobotInputData):
        
        self.tag = self.tags_q.get()
        offset = get_median_tag_offset(self.tag)

        if offset is not None:
            self.dx, self.dy, self.dz = offset
            self.dz -= 0.3

            # Deadzone filtering
            if abs(self.dz) > 0.1: self.dz = 0
            if abs(self.dx) < 0.05: self.dx = 0
            if abs(self.dy) < 0.05: self.dy = 0

            # Camera (z,x,y) → Robot (x,y,z)
            dx = 0
            dy = self.dx
            dz = -self.dy
            rx = ry = rz = 0

            # === Normalize to max frame displacement
            max_step = PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG * INTERVAL_S
            vec = np.array([dx, dy, dz])
            norm = np.linalg.norm(vec)
            if norm > 0:
                scale = min(1.0, max_step / norm)
                dx, dy, dz = (vec * scale).tolist()

            return SingleCartesianJogAction(
                    dx=dx, dy=dy, dz=dz,
                    rx=rx, ry=ry, rz=rz,
                    frame="WRF",
                    speed_pct=self.jog_control[0],
                    interval_s=INTERVAL_S,
                    shared_string=self.shared_string
                )


        else:
            self.dx, self.dy, self.dz = (0, 0, 0)
            return DummyAction(robot_data.position)

    def to_dict(self):
        # build per-tag entries
        tags_dict = {
            tag.tag_id: {
                "center": (float(tag.center[0]), float(tag.center[1])),
                "pose": {
                    "x": float(tag.pose_t[0][0]),
                    "y": float(tag.pose_t[1][0]),
                    "z": float(tag.pose_t[2][0]),
                }
            }
            for tag in self.tag
        }

        # now merge last_offset at top level
        return {
            "last_offset": {"dx": self.dx, "dy": self.dy, "dz": self.dz},
            **tags_dict
        }

# === Full Features-based IBVS Reactor (with termination & logging) ===
from typing import Optional, List, Any, Dict, Tuple
import numpy as np
import time, json, os, re
from types import SimpleNamespace

# 依赖你工程中的类型/基类/动作
# from Reactor import Reactor, INTERVAL_S
# from action import SingleCartesianJogAction, DummyAction

Uv = Tuple[float, float]

# modified IBVSReactor using tools/tag_utils
import os
import time
import json
from types import SimpleNamespace
from typing import Optional, Any, List, Tuple, Dict

import numpy as np

# project-specific imports (assume these exist in your project)
# from your_action_module import DummyAction, SingleCartesianJogAction, INTERVAL_S, Uv
# (keep your existing imports in the file; below are the tag utils we now use)

from tools.tag_tools import get_id, build_obs_from_items, normalize_item, get_corners, get_pose_t
from spatialmath.base import trlog

# Ensure INTERVAL_S, DummyAction, SingleCartesianJogAction, Uv are imported in your file scope
# e.g. from StatelessAction import DummyAction, SingleCartesianJogAction
# and INTERVAL_S is defined elsewhere in your project

class IBVSReactor(Reactor):
    """
    Features-based IBVS（AprilTag 四角点，最多8点）
    (docstring same as yours)
    """

    def __init__(self,
                 tags_q,
                 jog_control,
                 shared_string,
                 # 控制增益
                 lam: float = 0.5,           # m/s per normalized error
                 mu: float = 1e-3,           # DLS damping
                 target_ids: Optional[List[int]] = None,
                 desired_uv: Optional[Any] = None,
                 # 控制/安全参数
                 pixel_dead_px: float = 4.0,
                 max_step_m: float = 0.005,  # 每步平移上限（m）
                 max_rot_rad: float = 0.01,  # 每步转角上限（rad）
                 use_depth: bool = False,
                 depth_est: float = 0.5,
                 depth_alpha: float = 0.2,   # 深度 EMA 系数
                 # 坐标/单位
                 angles_in_rad: bool = False,  # 你的 cartesian_jog 用度 -> False
                 P_override: Optional[np.ndarray] = None,  # 自定义 3×3 映射矩阵
                 # 终止条件
                 stop_on_converge: bool = True,
                 converge_px_rms: float = 2.0,
                 converge_frames: int = 10,
                 timeout_s: Optional[float] = None,
                 stagnation_eps: float = 1e-3,
                 stagnation_frames: int = 30,
                 # 轴掩码（便于排障，1=启用，0=关闭）
                 axis_mask_lin: Tuple[int,int,int] = (1,1,1),
                 axis_mask_ang: Tuple[int,int,int] = (1,1,1),
                 rvec_mask: Tuple[int,int,int] = (1,1,1),
                 
                 # 日志
                 log_path: Optional[str] = None,
                 log_every: int = 1
                 ):
        super().__init__()
        self.tags_q = tags_q
        self.jog_control = jog_control
        self.shared_string = shared_string

        # 基本控制
        self.lam = float(lam)
        self.mu  = float(mu)
        self.dt = INTERVAL_S
        self.target_ids = target_ids

        # desired features（N 对 (u*,v*)）
        self.desired_uv = desired_uv
        self.desired_rvec = np.zeros(3, dtype=float)  # 预留

        # 相机内参
        self.K = None
        try:
            param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
            self.K = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"),
                                delimiter=',').astype(float)
        except Exception as exc:
            print("[IBVS] Warning: cannot load camera intrinsics:", exc)
        
        self.P = np.array([[-1.0,  0.0, 0.0],
                            [0.0, -1.0, 0.0],
                            [0.0,  0.0, 1.0]], dtype=float) if P_override is None else np.array(P_override, dtype=float)
            
        # 参数
        self.pixel_dead_px = float(pixel_dead_px)
        self.max_step_m = float(max_step_m)
        self.max_rot_rad = float(max_rot_rad)
        self.use_depth = bool(use_depth)
        self.depth_est = float(depth_est)
        self.depth_alpha = float(depth_alpha)
        self.angles_in_rad = bool(angles_in_rad)

        # 终止状态
        self.stop_on_converge   = bool(stop_on_converge)
        self.converge_px_rms    = float(converge_px_rms)
        self.converge_frames    = int(converge_frames)
        self.timeout_s          = None if (timeout_s is None) else float(timeout_s)
        self.stagnation_eps     = float(stagnation_eps)
        self.stagnation_frames  = int(stagnation_frames)

        self._conv_count  = 0
        self._stagn_count = 0
        self._prev_enorm  = None
        self._t0          = time.time()
        self._completed   = False

        # 轴掩码
        self.axis_mask_lin = np.array(axis_mask_lin, dtype=int).clip(0,1)
        self.axis_mask_ang = np.array(axis_mask_ang, dtype=int).clip(-1,1)
        self.rvec_mask = np.array(rvec_mask,dtype=int).clip(0,1)
        self.rpy_gain = 0.03

        # 日志
        self.log_path   = log_path
        self.log_every  = max(1, int(log_every))
        self._tick      = 0

        # telemetry
        self._last_centroid_uv: Optional[Uv] = None
        self._last_pts_obs: List[Uv] = []
        self._last_pts_des: List[Uv] = []
        self._last_e = (0.0, 0.0)  # e_norm RMS
        self._last_v_tcp = (0.0, 0.0, 0.0)
        self._last_omega_cam = (0.0, 0.0, 0.0)
        self._last_omega_tcp = (0.0, 0.0, 0.0)
    
    def reset(self, robot_data: RobotInputData):
        self._tick = 0
        self._completed = 0
        self.set_desired_pose_from_current(robot_data=robot_data)
        pass

    # ---------- setters ----------
    def set_desired(self, desired_uv: Any):
        """ desired_uv: list of (u*,v*) with length == #observed features """
        self.desired_uv = desired_uv

    def set_desired_rvec(self, rvec):
        """ optional desired rotation vector in camera frame (Rodrigues rvec) """
        try:
            self.desired_rvec = np.array(rvec, dtype=float).flatten()
        except Exception:
            self.desired_rvec = np.zeros(3, dtype=float)
    
    def set_axis_mask(self, lin:tuple[int,int,int], ang:tuple[int,int,int]):
        self.axis_mask_lin = (lin[0],lin[1],lin[2])
        self.axis_mask_ang = (ang[0],ang[1],ang[2])

    def is_completed(self) -> bool:
        return bool(self._completed)

    # ---------- logging ----------
    def _log(self, kind: str, info: dict):
        if not self.log_path:
            return
        if (self._tick % self.log_every) != 0 and kind == "move":
            return
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True) if os.path.dirname(self.log_path) else None
            rec = {"tick": int(self._tick), "kind": str(kind)}
            rec.update(info or {})
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception:
            pass

    # ---------- simplified tag read (uses tag_utils) ----------
    def _read_tags(self):
        """Non-blocking read from tags_q; return None if nothing available."""
        try:
            return self.tags_q.get_nowait()
        except Exception:
            return None

    # ---------- core IBVS computation (unchanged) ----------
    def _compute_ibvs_twist(self, obs_uvs, des_uvs, Z_vals, K, lam=None, mu=1e-3):
        """
        obs_uvs: [(u,v)...]    观测特征
        des_uvs: [(u*,v*)...]  期望特征（长度与 obs 一致）
        Z_vals : [Z_i...]      每点深度（m），未知则用 self.depth_est
        K: 3x3 内参
        return v_cam: [vx,vy,vz, wx,wy,wz] 相机系 (m/s, rad/s)
        """
        fx, fy, cx, cy = float(K[0,0]), float(K[1,1]), float(K[0,2]), float(K[1,2])
        n = len(obs_uvs)
        assert len(des_uvs) >= n and len(Z_vals) >= n, "length mismatch"

        e = np.zeros(2*n, dtype=float)
        L = np.zeros((2*n, 6), dtype=float)

        for i, ((u,v), (us,vs), Z) in enumerate(zip(obs_uvs, des_uvs, Z_vals)):
            Zi = max(1e-3, float(Z))
            x  = (float(u)  - cx) / fx
            y  = (float(v)  - cy) / fy
            xs = (float(us) - cx) / fx
            ys = (float(vs) - cy) / fy

            e[2*i]   = x - xs
            e[2*i+1] = y - ys

            L[2*i:2*i+2, :] = np.array([
                [-1.0/Zi,    0.0,     x/Zi,     x*y,      -(1+x*x),   y],
                [ 0.0,     -1.0/Zi,   y/Zi,   1.0+y*y,     -x*y,     -x]
            ], dtype=float)

        lam = self.lam if lam is None else float(lam)
        try:
            LTL = L.T @ L
            v_cam = - lam * (np.linalg.inv(LTL + mu*np.eye(6)) @ L.T @ e)
        except np.linalg.LinAlgError:
            v_cam = - lam * (np.linalg.pinv(L) @ e)

        # 存相机系角速度的“每步”值（仅 telemetry）
        self._last_omega_cam = (float(v_cam[3]*self.dt), float(v_cam[4]*self.dt), float(v_cam[5]*self.dt))
        return v_cam
    
    def set_desired_pose_from_current(self, robot_data):
        """
        读取 robot_data 当前的末端姿态，并保存为 desired_rvec。
        - robot_data: RobotInputData（包含关节位置）
        - 输出: 更新 self.desired_rvec
        """
        # 1) 读取当前关节角度（rad）
        q_rad = [PAROL6_ROBOT.STEPS2RADS(int(robot_data.position[i]), i) for i in range(6)]
        T_current = PAROL6_ROBOT.robot.fkine(q_rad)

        # 2) 取旋转矩阵
        R_current = T_current.R

        # 3) 转成 Rodrigues 向量
        rvec, _ = cv2.Rodrigues(R_current)

        # 4) 存到 self.desired_rvec
        self.desired_rvec = rvec.flatten()

        # 可选：日志
        print(f"[IBVS] desired_rvec set from current pose: {self.desired_rvec}")

    # ---------- main ----------
    def plan(self, robot_data):
        try:
            self._tick += 1

            tags = self._read_tags()
            if (tags is None):
                print("Dummy Tag break")
                return DummyAction(robot_data.position)
            
            if (self.K is None):
                print("Dummy No K")
                return DummyAction(robot_data.position)

            # Normalize tags payload to iterable list
            tags_list = list(tags) if isinstance(tags, (list, tuple)) else [tags]

            # filter by id (if target_ids set)
            if self.target_ids:
                idset = set(self.target_ids)
                usable = [t for t in tags_list if (get_id(t) in idset)]
            else:
                usable = tags_list

            usable = usable[:2]  # 最多两个 tag
            if not usable:
                self._log("no_usable_tags", {})
                self._last_pts_obs, self._last_pts_des = [], []
                self._last_centroid_uv = None
                print("Dummy No Tags")
                return DummyAction(robot_data.position)

            # Build observation list (obs_uvs per corner flattened) and Z_vals using tag_utils
            obs_uvs, Z_vals = build_obs_from_items(usable, use_pose_depth=self.use_depth, depth_default=self.depth_est)

            # If no corners found -> bail
            if len(obs_uvs) == 0:
                self._log("no_corners", {})
                self._last_pts_obs, self._last_pts_des = [], []
                self._last_centroid_uv = None
                print("Dummy corners")
                return DummyAction(robot_data.position)

            # compute per-tag z list for EMA (collect one z per tag if available)
            tag_z_list = []
            for t in usable:
                pt = get_pose_t(t)
                if pt is not None:
                    try:
                        tag_z_list.append(float(pt[2]))
                    except Exception:
                        pass
            # If none found, tag_z_list remains empty and will not update depth_est

            # 质心（用于可视化）
            u_c = float(np.mean([p[0] for p in obs_uvs]))
            v_c = float(np.mean([p[1] for p in obs_uvs]))
            self._last_centroid_uv = (u_c, v_c)
            self._last_pts_obs = obs_uvs

            # desired 列表（长度匹配）
            fx, fy = float(self.K[0,0]), float(self.K[1,1])
            cx, cy = float(self.K[0,2]), float(self.K[1,2])
            N = len(obs_uvs)

            if (isinstance(self.desired_uv, (list, tuple))
                and len(self.desired_uv) >= N
                and isinstance(self.desired_uv[0], (list, tuple, np.ndarray))
                and len(self.desired_uv[0]) >= 2):
                des_list = [tuple(map(float, uv)) for uv in self.desired_uv[:N]]
            elif (isinstance(self.desired_uv, (list, tuple))
                  and len(self.desired_uv) == 2
                  and all(isinstance(x, (int, float)) for x in self.desired_uv)):
                des_list = [tuple(map(float, self.desired_uv))] * N
            else:
                des_list = [(cx, cy)] * N  # 兜底：居中

            self._last_pts_des = des_list

            # 像素 RMS
            diffs = np.array(obs_uvs, dtype=float) - np.array(des_list, dtype=float)
            rms_u = float(np.sqrt(np.mean(diffs[:,0]**2)))
            rms_v = float(np.sqrt(np.mean(diffs[:,1]**2)))
            self._last_e = (rms_u/fx, rms_v/fy)  # 记录 e_norm RMS

            # ---------- 终止条件：收敛/停滞/超时 ----------
            if rms_u < self.converge_px_rms and rms_v < self.converge_px_rms:
                self._conv_count += 1
            else:
                self._conv_count = 0
            if self.stop_on_converge and self._conv_count >= self.converge_frames:
                self._completed = True
                self._log("converged", {"rms_u": rms_u, "rms_v": rms_v, "frames": self._conv_count})
                print("Dummy Converged")
                return DummyAction(robot_data.position)

            enorm = float(np.sqrt((rms_u/fx)**2 + (rms_v/fy)**2))
            if self._prev_enorm is not None:
                if abs(self._prev_enorm - enorm) < self.stagnation_eps:
                    self._stagn_count += 1
                else:
                    self._stagn_count = 0
            self._prev_enorm = enorm
            if self._stagn_count >= self.stagnation_frames:
                self._completed = True
                self._log("stagnation", {"enorm": enorm, "frames": self._stagn_count})
                print("DummyStagnant")
                return DummyAction(robot_data.position)

            if (self.timeout_s is not None) and ((time.time() - self._t0) > self.timeout_s):
                self._completed = True
                self._log("timeout", {"seconds": self.timeout_s})
                print("Dummy Timeout")
                return DummyAction(robot_data.position)
            # ---------- 终止条件：结束 ----------

            # Deadzone（不终止，只是不动）
            if (rms_u < self.pixel_dead_px) and (rms_v < self.pixel_dead_px):
                self._log("deadzone", {"rms_u": rms_u, "rms_v": rms_v, "px_dead": self.pixel_dead_px})
                print("Dummy Deadzone")
                return DummyAction(robot_data.position)

            # 深度 EMA（若拿到了新的 z）
            if tag_z_list:
                med_z = float(np.median(tag_z_list))
                self.depth_est = (1.0 - self.depth_alpha) * float(self.depth_est) + self.depth_alpha * med_z

            # 计算 v_cam（m/s, rad/s）
            v_cam = self._compute_ibvs_twist(
                obs_uvs=obs_uvs,
                des_uvs=des_list,
                Z_vals=Z_vals if self.use_depth else [self.depth_est]*N,
                K=self.K,
                lam=self.lam,
                mu=self.mu
            )

            # 积分为每步增量
            v_lin_cam = np.array(v_cam[:3], dtype=float)
            omg_cam   = np.array(v_cam[3:], dtype=float)
            d_cam_lin = v_lin_cam * self.dt                  # m/step
            d_cam_ang = omg_cam * self.dt                    # rad/step

            # 步长限幅
            lin_norm = float(np.linalg.norm(d_cam_lin))
            if lin_norm > 0 and lin_norm > self.max_step_m:
                d_cam_lin *= (self.max_step_m / lin_norm)
            d_cam_ang = np.clip(d_cam_ang, -self.max_rot_rad, self.max_rot_rad)

            # 映射相机系 -> TCP 系，并应用轴掩码
            d_tcp_lin = self.P @ d_cam_lin
            d_tcp_ang = self.P @ d_cam_ang

            d_tcp_lin *= self.axis_mask_lin.astype(float)
            d_tcp_ang *= self.axis_mask_ang.astype(float)

            # 当前 TCP 姿态 (用 robot_data.position → 正向运动学)
            q_rad = [PAROL6_ROBOT.STEPS2RADS(int(robot_data.position[i]), i) for i in range(6)]
            T_current = PAROL6_ROBOT.robot.fkine(q_rad)
            R_current = T_current.R    # 3x3 rotation

            # 基准姿态
            R_des = (SE3.Rx(self.desired_rvec[0]) *
                    SE3.Ry(self.desired_rvec[1]) *
                    SE3.Rz(self.desired_rvec[2]))
            R_err = R_des.R @ R_current.T

            # 转成旋转向量（Rodrigues）
            rvec_err, _ = cv2.Rodrigues(R_err)
            rvec_err = rvec_err.flatten()

            # 只吸附 rx，丢掉 rz，ry 保持自由
            rx_corr = rvec_err[0]
            ry_corr = rvec_err[1]
            rz_corr = rvec_err[2]
            if abs(rx_corr) > getattr(self, "rpy_tol", np.deg2rad(2.0)):
                d_tcp_ang[0] += self.rpy_gain * rx_corr * self.rvec_mask[0] # rx 吸附
                d_tcp_ang[1] += self.rpy_gain * ry_corr * self.rvec_mask[1] # ry 吸附
                d_tcp_ang[2] += self.rpy_gain * rz_corr * self.rvec_mask[2]  # rz 吸附

            # 解包为标量
            dx, dy, dz = map(float, d_tcp_lin.tolist())
            rx, ry, rz = map(float, d_tcp_ang.tolist())


            # 若 Action 需要度，转换
            if not self.angles_in_rad:
                rx, ry, rz = np.rad2deg([rx, ry, rz]).tolist()

            # 记录
            self._last_v_tcp = (dx, dy, dz)
            self._last_omega_tcp = (rx, ry, rz)

            # UI Telemetry（短串）
            dbg = (f"IBVS-F N={N} centroid=({u_c:.1f},{v_c:.1f}) "
                   f"rms_px=({rms_u:.1f},{rms_v:.1f}) "
                   f"dx/dy/dz_mm=({dx*1000:.2f},{dy*1000:.2f},{dz*1000:.2f}) "
                   f"rot_tcp_{'deg' if not self.angles_in_rad else 'rad'}=({rx:.4f},{ry:.4f},{rz:.4f}) "
                   f"Zest={self.depth_est:.3f}")
            try:
                if self.shared_string is not None:
                    self.shared_string.value = dbg.encode()[:120]
            except Exception:
                pass

            # speed percent
            try:
                speed_pct = int(self.jog_control[0]) if (self.jog_control is not None and len(self.jog_control) > 0) else 50
            except Exception:
                speed_pct = 50

            # 日志（move）
            self._log("move", {
                "N": N,
                "rms_u": round(rms_u, 3), "rms_v": round(rms_v, 3),
                "e_norm_x": round(self._last_e[0], 6), "e_norm_y": round(self._last_e[1], 6),
                "Zest": round(self.depth_est, 4),
                "dx": dx, "dy": dy, "dz": dz,
                "rx": rx, "ry": ry, "rz": rz,
                "speed_pct": int(speed_pct),
            })

            return SingleCartesianJogAction(
                dx=dx, dy=dy, dz=dz,
                rx=rx, ry=ry, rz=rz,
                frame="TRF",
                speed_pct=speed_pct,
                interval_s=self.dt,
                shared_string=self.shared_string
            )

        except Exception as exc:
            try:
                if self.shared_string is not None:
                    self.shared_string.value = f"IBVS plan error: {exc}".encode()[:120]
            except Exception:
                pass
            self._log("exception", {"msg": str(exc)})
            print("IBVS plan exception:", exc)
            return DummyAction(robot_data.position)

    # ---------- to_dict ----------
    def to_dict(self) -> Dict[str, Any]:
        cxcy = None if self._last_centroid_uv is None else {
            "u": round(self._last_centroid_uv[0], 2),
            "v": round(self._last_centroid_uv[1], 2)
        }
        return {
            "centroid_uv": cxcy,
            "obs_pts": [{"u": round(u, 2), "v": round(v, 2)} for (u, v) in self._last_pts_obs],
            "des_pts": [{"u": round(u, 2), "v": round(v, 2)} for (u, v) in self._last_pts_des],
            "e_norm_rms": {"x": round(self._last_e[0], 6), "y": round(self._last_e[1], 6)},
            "v_tcp": {"dx_m": round(self._last_v_tcp[0], 6),
                      "dy_m": round(self._last_v_tcp[1], 6),
                      "dz_m": round(self._last_v_tcp[2], 6)},
            "omega_cam_step": {"wx": round(self._last_omega_cam[0], 6),
                               "wy": round(self._last_omega_cam[1], 6),
                               "wz": round(self._last_omega_cam[2], 6)},
            "omega_tcp_step": {"rx": round(self._last_omega_tcp[0], 6),
                               "ry": round(self._last_omega_tcp[1], 6),
                               "rz": round(self._last_omega_tcp[2], 6)},
            "lam": self.lam, "mu": self.mu,
            "pixel_dead_px": self.pixel_dead_px,
            "use_depth": self.use_depth,
            "max_step_m": self.max_step_m,
            "max_rot_rad": self.max_rot_rad,
            "depth_est": round(self.depth_est, 4),
            "completed": self._completed
        }




# reactor_pbvs_coarse_plan.py
import time, queue
import numpy as np
import cv2
from dataclasses import dataclass
from spatialmath import SE3

from tools.shared_struct import RobotInputData
import tools.PAROL6_ROBOT as PAROL6_ROBOT
from action import Move2JointsAction, SingleCartesianJogAction, DummyAction
from Reactor import Reactor   # 继承

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
    WAIT_TAG → ROT_ALIGN(只姿态) → XY_SLIDE(只位置) → PUSH_TRF(沿工具Z直推) → DONE
    - 从 robot_data.position 取 q
    - 不改 Vision：从 tags_q 取 detections，自行做多标签 PnP + EMA/slerp
    - camera=工具轴：默认 T_tcp_cam = I
    - 每次 plan() 只返回一个 Action
    """
    def __init__(self, tags_q, K: np.ndarray, dist: np.ndarray | None,
                 T_tcp_cam: SE3 | None = None, params: PBVSParams = PBVSParams()):
        super().__init__()
        self.tags_q = tags_q
        self.K = np.asarray(K, dtype=np.float64)
        self.dist = None if dist is None else np.asarray(dist, dtype=np.float64)
        self.T_tcp_cam = T_tcp_cam if T_tcp_cam is not None else SE3()  # camera=工具轴
        self.p = params

        self.state = "WAIT_TAG"
        self._pose_ema: SE3 | None = None
        self._last_pose_ts = 0.0
        self.T_w_goal: SE3 | None = None
        self._push_total = 0
        self._push_done = 0
        
    def reset(self, robot_data: RobotInputData):
        pass
    # === 核心 ===
    def plan(self, robot_data: RobotInputData):
        q_now = self._q(robot_data)
        T_w_tcp = PAROL6_ROBOT.robot.fkine(q_now)
        now = time.time()

        if self.state == "WAIT_TAG":
            T_cam_tag = self._fused_pose_from_queue()
            fresh = (T_cam_tag is not None) and ((now - self._last_pose_ts)*1000.0 <= self.p.fresh_ms)
            if not fresh:
                return DummyAction(robot_data.position)

            T_w_tag = T_w_tcp * self.T_tcp_cam * T_cam_tag
            z_tool_des = -T_w_tag.R[:,2]
            R_align = self._R_from_z(z_tool_des, self.p.yaw_only)
            T_tag_tool = SE3.Rt(R_align, [0,0,self.p.standoff])
            self.T_w_goal = T_w_tag * T_tag_tool

            self.state = "ROT_ALIGN"
            T_rot_goal = SE3.Rt(self.T_w_goal.R, T_w_tcp.t)
            q1 = self._ik(T_rot_goal, q_now)
            return Move2JointsAction(q1, v_pct=40.0)

        if self.state == "ROT_ALIGN":
            self.state = "XY_SLIDE"
            T_xy_goal = SE3.Rt(self.T_w_goal.R, [self.T_w_goal.t[0], self.T_w_goal.t[1], T_w_tcp.t[2]])
            q2 = self._ik(T_xy_goal, q_now)
            return Move2JointsAction(q2, v_pct=35.0)

        if self.state == "XY_SLIDE":
            self.state = "PUSH_TRF"
            self._push_total = max(1, int(abs(self.p.push_dist)/self.p.step_trf))
            self._push_done = 0
            return self._one_push_step()

        if self.state == "PUSH_TRF":
            if self._push_done < self._push_total:
                return self._one_push_step()
            self.state = "DONE"
            return DummyAction(robot_data.position)

        if self.state == "DONE":
            return DummyAction(robot_data.position)

        return DummyAction(robot_data.position)

    # --- 直推一步：返回 SingleCartesianJogAction ---
    def _one_push_step(self):
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

    # --- Vision 融合 ---
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
            c = self._corners(d)
            m = self._margin(d)
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

        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return self._pose_ema

        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.dist)
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
            t_blend = (1.0 - self.p.alpha_t) * self._pose_ema.t + self.p.alpha_t * T_new.t
            R_blend = self._slerp(self._pose_ema.R, T_new.R, self.p.alpha_r)
            self._pose_ema = SE3.Rt(R_blend, t_blend)
        return self._pose_ema

    # --- 机器人/IK/几何 ---
    @staticmethod
    def _q(robot_data: RobotInputData) -> np.ndarray:
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
            yaw = float(np.arctan2(z[1], z[0]))
            return _SE3.Rz(yaw).R
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
