import time
from abc import ABC, abstractmethod
from tools.shared_struct import RobotInputData, RobotOutputData, check_elements
import numpy as np
from typing import List, Dict, Any
from multiprocessing import Array
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
from action import SingleJointJogAction, SingleCartesianJogAction, HomeRobotAction, EnableRobotAction, DisableRobotAction, DummyAction, ClearErrorAction
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
        self.counter = 0

    def giveCommand(self, robot_data: RobotInputData) -> Action:
        """
        This would be called for every 10ms. It will parse in the cmd_data and modify the shared memory inside cmd_data
        Then return the cmd_data.pack() -> byte list
        """
        return self.plan(robot_data=robot_data)
    
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

import os
class IBVSReactor(Reactor):
    """
    Eye-in-Hand IBVS reactor that uses (two) AprilTags' image centers.
    - tags_q: multiprocessing.Queue or queue.Queue that yields a list of tag detections (same objects as Vision.py)
    - jog_control: multiprocessing.Array (same as GUI usage) 用于取速度百分比
    - shared_string: multiprocessing.Array 用于显示日志
    - depth_est: 初始深度估计 (m)
    - lam: 控制增益
    - frame: "TRF" or "WRF" — Eye-in-Hand 推荐 "TRF"
    - target_ids: Optional[List[int]] 如果提供，只使用这些 id（前两个）来计算中心
    """
    def __init__(self, tags_q, jog_control, shared_string,
                 depth_est: float = 0.5,
                 lam: float = 0.6,
                 frame: str = "TRF",
                 target_ids: Optional[List[int]] = None):
        super().__init__()
        self.tags_q = tags_q
        self.jog_control = jog_control
        self.shared_string = shared_string
        self.depth_est = float(depth_est)
        self.lam = float(lam)
        self.dt = INTERVAL_S
        self.frame = frame.upper()
        self.target_ids = target_ids

        # load camera intrinsics from repo same path as Vision.py
        try:
            param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
            self.K = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), delimiter=',')
        except Exception as exc:
            self.K = None
            print("[IBVS] Warning: cannot load camera intrinsics:", exc)

        # debug / telemetry
        self._last_e = (0.0, 0.0)
        self._last_v = (0.0, 0.0, 0.0)
        self._last_uv = None

    def _read_tags(self):
        """Try get latest tag list without blocking; return list or None"""
        try:
            tags = self.tags_q.get_nowait()
            return tags
        except Exception:
            return None

    def plan(self, robot_data):
        """
        Read tags -> compute pixel centroid -> IBVS -> SingleCartesianJogAction.
        Compatible with:
        - tags_q producing list of AprilTag objects (with .center, .pose_t)
        - tags_q producing list of tuples (id,u,v) or (u,v)
        - tags_q producing a single tuple (id,u,v) or (u,v)
        - mixed lists
        """
        # 1) read tags snapshot (non-blocking via helper)
        tags = self._read_tags()  # may be None, a list, or a single tuple

        # quick exit if no data or no intrinsics
        if (not tags) or (self.K is None):
            return DummyAction(robot_data.position)

        # normalize tags into a list
        if not isinstance(tags, (list, tuple)):
            tags_list = [tags]
        else:
            tags_list = list(tags)

        # 2) pick usable entries (filter by target_ids if provided) and keep original order
        usable = []
        if self.target_ids:
            ids_set = set(self.target_ids)
            for t in tags_list:
                tid = None
                if hasattr(t, "tag_id"):
                    tid = getattr(t, "tag_id", None)
                elif isinstance(t, (tuple, list)) and len(t) >= 1:
                    try:
                        tid = int(t[0])
                    except Exception:
                        tid = None
                if tid in ids_set:
                    usable.append(t)
            usable = usable[:2]
        else:
            usable = tags_list[:2]

        if len(usable) < 1:
            return DummyAction(robot_data.position)

        # 3) extract pixel centers from usable items (support object and tuple formats)
        centers = []
        bad_cnt = 0
        for t in usable:
            parsed = False
            if hasattr(t, "center"):
                try:
                    cx = float(t.center[0])
                    cy = float(t.center[1])
                    centers.append((cx, cy))
                    parsed = True
                except Exception:
                    parsed = False
            if not parsed and isinstance(t, (tuple, list)):
                try:
                    if len(t) >= 3:
                        # (id, u, v)
                        cx = float(t[1]); cy = float(t[2])
                        centers.append((cx, cy)); parsed = True
                    elif len(t) == 2:
                        # (u, v)
                        cx = float(t[0]); cy = float(t[1])
                        centers.append((cx, cy)); parsed = True
                except Exception:
                    parsed = False
            if not parsed:
                bad_cnt += 1

        if bad_cnt:
            # short-lived warning to help debug mixed inputs (optional)
            try:
                import logging
                logging.getLogger(__name__).debug("IBVSReactor.plan: skipped %d unusable tag items", bad_cnt)
            except Exception:
                pass

        if not centers:
            # nothing usable
            self._last_uv = None
            return DummyAction(robot_data.position)

        # centroid (works for 1 or 2 points)
        u = float(np.mean([c[0] for c in centers]))
        v = float(np.mean([c[1] for c in centers]))
        self._last_uv = (u, v)

        # 4) intrinsics
        fx, fy = float(self.K[0,0]), float(self.K[1,1])
        cx, cy = float(self.K[0,2]), float(self.K[1,2])

        # normalized image coord (desired is image center -> (0,0) in normalized coords)
        x = (u - cx) / fx
        y = (v - cy) / fy
        self._last_e = (x, y)

        # 5) estimate depth Z from pose_t if available (only from object entries)
        Z_vals = []
        for t in usable:
            if hasattr(t, "pose_t"):
                try:
                    z = float(np.array(t.pose_t).flatten()[2])
                    Z_vals.append(z)
                except Exception:
                    pass
        if Z_vals:
            Z = float(np.median(Z_vals))
            Z = max(Z, 1e-3)
            # simple low-pass update of running estimate
            self.depth_est = 0.8 * self.depth_est + 0.2 * Z
        else:
            Z = max(1e-3, float(self.depth_est))

        # 6) IBVS interaction matrix and control
        L = np.array([
            [-1.0/Z,    0.0,    x/Z,    x*y,  -(1 + x*x),   y],
            [  0.0, -1.0/Z,    y/Z,  1 + y*y,   -x*y,     -x],
        ], dtype=float)

        try:
            L_pinv = np.linalg.pinv(L)
            e = np.array([[x],[y]])
            v_c = -self.lam * (L_pinv @ e)   # (6,1)
            vx, vy, vz, wx, wy, wz = v_c.flatten().tolist()
        except Exception as exc:
            if self.shared_string is not None:
                try: self.shared_string.value = f"IBVS: pinv failed {exc}".encode()[:120]
                except: pass
            return DummyAction(robot_data.position)

        self._last_v = (vx, vy, vz)

        # 7) only use translation for now (camera frame)
        dx, dy, dz = vx * self.dt, vy * self.dt, vz * self.dt

        # 8) safety: limit per-loop linear displacement
        max_step = PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG * self.dt
        vec = np.array([dx, dy, dz], dtype=float)
        nrm = float(np.linalg.norm(vec))
        if nrm > max_step and nrm > 0:
            vec *= (max_step / nrm)
            dx, dy, dz = vec.tolist()

        # 9) deadzone to avoid jitter
        PIX_DEAD_NORM = 1e-3
        if abs(x) < PIX_DEAD_NORM and abs(y) < PIX_DEAD_NORM:
            return DummyAction(robot_data.position)

        # 10) return action (TRF/WRF per self.frame)
        action = SingleCartesianJogAction(
            dx=dx, dy=-dz, dz=dy,
            rx=0.0, ry=0.0, rz=0.0,
            frame=self.frame,
            speed_pct=self.jog_control[0] if self.jog_control is not None else 50,
            interval_s=self.dt,
            shared_string=self.shared_string
        )
        return action


    def to_dict(self) -> Dict[str, Any]:
        return {
            "last_uv": {"u": None if self._last_uv is None else round(self._last_uv[0], 2),
                        "v": None if self._last_uv is None else round(self._last_uv[1], 2)},
            "e_norm": {"x": round(self._last_e[0], 5), "y": round(self._last_e[1], 5)},
            "v_lin": {"vx": round(self._last_v[0], 5), "vy": round(self._last_v[1], 5), "vz": round(self._last_v[2], 5)},
            "lam": self.lam,
            "Z_est": round(self.depth_est, 4),
            "frame": self.frame,
        }
        
