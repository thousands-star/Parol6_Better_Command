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

class IBVSReactor(Reactor):
    """
    Eye-in-Hand IBVS (Image-Based Visual Servoing).
    每个周期通过图像误差计算一次笛卡尔微小位移，返回 SingleCartesianJogAction。
    """
    def __init__(self, uv_q, jog_control, shared_string,
                 depth_est: float = 0.5,   # 目标到相机的初始深度估计(米)
                 lam: float = 0.6,         # 控制增益
                 frame: str = "TRF"):      # 在 TRF（工具坐标）下执行最自然
        super().__init__()
        self.uv_q = uv_q
        self.jog_control = jog_control
        self.shared_string = shared_string
        self.K = np.loadtxt(r"C:\Users\Public\fyp\PAROL-commander-software-main\GUI\files\tools\Camera\param\camera_intrinsic_matrix.csv", delimiter=",")  # 你前面标定好的内参
        
        self.depth_est = float(depth_est)
        self.lam = float(lam)
        self.dt = INTERVAL_S
        self.frame = frame.upper()
        self.last_uv = None  # (u,v)
        # 便于调试监控
        self._last_e = (0.0, 0.0)
        self._last_v = (0.0, 0.0, 0.0)

    def plan(self, robot_data: RobotInputData):
        """
        读入最新 (u,v)，计算 IBVS 控制律，转成一次微小位移，返回 SingleCartesianJogAction。
        """
        # 1) 读取最新的像素坐标 (u,v)，若队列为空则复用上次
        try:
            self.last_uv = self.uv_q.get_nowait()
        except Exception:
            pass

        if self.last_uv is None or self.K is None:
            # 没有目标就发一个“空动作”把速度清零
            return DummyAction(robot_data.position)

        u, v = float(self.last_uv[0]), float(self.last_uv[1])
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        # 2) 归一化图像坐标（期望为 0,0）
        x = (u - cx) / fx
        y = (v - cy) / fy
        e = np.array([[x], [y]])  # e = s - s*, s*=(0,0)
        self._last_e = (x, y)

        # 3) Interaction Matrix L (2x6)
        Z = max(1e-3, float(self.depth_est))
        L = np.array([
            [-1/Z,   0,   x/Z,   x*y, -(1 + x*x),    y],
            [  0 , -1/Z,  y/Z, 1 + y*y,   -x*y,     -x],
        ], dtype=float)

        # 4) 控制律 v_c = -λ L^+ e
        L_pinv = np.linalg.pinv(L)
        v_c = -self.lam * (L_pinv @ e)  # (6,1)
        vx, vy, vz, wx, wy, wz = v_c.flatten().tolist()
        self._last_v = (vx, vy, vz)

        # 只先做平移控制，把速度换成本周期的微小位移
        dx, dy, dz = vx * self.dt, vy * self.dt, vz * self.dt

        # 5) 安全限幅：不超过每周期最大线速度对应的步长
        max_step = PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG * self.dt
        vec = np.array([dx, dy, dz], dtype=float)
        nrm = float(np.linalg.norm(vec))
        if nrm > max_step and nrm > 0:
            vec *= (max_step / nrm)
            dx, dy, dz = vec.tolist()

        # 可选：极小误差死区，避免抖动
        if abs(x) < 1e-3 and abs(y) < 1e-3:
            return DummyAction(robot_data.position)

        # 6) 返回一次性的“笛卡尔微移” Action，由 commander 执行
        return SingleCartesianJogAction(
            dx=dx, dy=dy, dz=dz,
            rx=0.0, ry=0.0, rz=0.0,       # 先不控姿态，后续可加入 wx,wy,wz
            frame=self.frame,             # Eye-in-Hand 下用 TRF 更合适
            speed_pct=self.jog_control[0],
            interval_s=self.dt,
            shared_string=self.shared_string
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "e_norm": {"x": round(self._last_e[0], 5), "y": round(self._last_e[1], 5)},
            "v_lin":  {"vx": round(self._last_v[0], 4), "vy": round(self._last_v[1], 4), "vz": round(self._last_v[2], 4)},
            "lam": self.lam,
            "Z_est": self.depth_est,
            "frame": self.frame,
        }
        
