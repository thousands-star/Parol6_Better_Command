import time
from abc import ABC, abstractmethod
from tools.shared_struct import RobotInputData, RobotOutputData, check_elements
import numpy as np
from typing import List, Dict, Any
from multiprocessing import Array
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
from tools.StatelessAction import move_joints, dummy_data, cartesian_jog
from tools.log_tools import nice_print_sections
import queue
from statistics import median           
from typing import List, Tuple, Optional

INTERVAL_S = 0.01
Robot_mode = "Dummy"

def cartesian_jog_step(cmd_data, robot_data, dx, dy, dz, jog_control, shared_string):
    # 1) current joint angles (rad)
    q0 = np.array([PAROL6_ROBOT.STEPS2RADS(robot_data.position[i], i) for i in range(6)])
    T0 = PAROL6_ROBOT.robot.fkine(q0)

    # 2) build the *unit* direction vector
    vec = np.array([dx, dy, dz])
    norm = np.linalg.norm(vec)
    if norm == 0:
        return  # nothing to do

    dir_vec = vec / norm

    # 3) compute frame‐independent step length from slider % and dt
    vel_pct = jog_control[0] / 100.0
    max_lin = PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG  # m/s
    step_len = max_lin * vel_pct * INTERVAL_S  # meters per frame

    # 4) apply small step
    if jog_control[2] == 1:  # WRF
        T0.t += dir_vec * step_len
        shared_string.value = b'Log: Cartesian WRF jog'
    else:                    # TRF
        new_pt = T0 * (dir_vec * step_len)
        T0.t = new_pt
        shared_string.value = b'Log: Cartesian TRF jog'

    # 5) solve IK for T0
    mask = [1,1,1,0,0,0]
    sol = PAROL6_ROBOT.robot.ikine_LMS(T0, q0=q0, ilimit=6, mask=mask)
    if not sol.success:
        shared_string.value = b'Warning: IK failed, skipping this step'
        return

    q1 = sol.q if hasattr(sol, 'q') else sol[0]
    delta_q = q1 - q0  # rad

    # 6) convert to step/sec based on direction and slider
    cmd_data.command.value = 123
    for i in range(6):
        # choose the max jog speed for this joint
        max_step_s = np.interp(jog_control[0],
                               [0, 100],
                               [PAROL6_ROBOT.Joint_min_jog_speed[i],
                                PAROL6_ROBOT.Joint_max_jog_speed[i]])
    for i in range(3):    
        cmd_data.speed[i] = int(np.sign(delta_q[i]) * max_step_s * 0.5)

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

    def giveCommand(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        """
        This would be called for every 10ms. It will parse in the cmd_data and modify the shared memory inside cmd_data
        Then return the cmd_data.pack() -> byte list
        """
        self.plan(robot_data=robot_data, cmd_data=cmd_data)
        if(
            cmd_data.gripper_data[4] == 1 or 
        cmd_data.gripper_data[4] == 2):
            
            cmd_data.gripper_data[4] = 0

        # if self.counter < 500:
        #     self.counter = self.counter + 1
        # else:
        #     Initprint = {
        #         "robot_data": robot_data.to_dict(),
        #         "cmd_data": 
        # cmd_data.to_dict()
        #     }

        #     nice_print_sections(Initprint)
        #     self.counter = 0

        return cmd_data.pack()
    
    @abstractmethod
    def plan(self, robot_data: RobotInputData, cmd_data: RobotOutputData) -> List[bytes]:
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


    def plan(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
            # Check if any of jog buttons is pressed
            result_joint_jog = check_elements(list(self.Joint_jog_buttons))
            result_cart_jog = check_elements(list(self.Cart_jog_buttons))

            ######################################################
            ######################################################
            InOut_in = robot_data.inout
            Position_in = robot_data.position

            
            # JOINT JOG (regular speed control) 0x123 # -1 is value if nothing is pressed
            if result_joint_jog != -1 and self.Buttons[2] == 0 and InOut_in[4] == 1: 
                Robot_mode = "Joint jog"
                
                joint_id = result_joint_jog % 6
                direction = 1 if result_joint_jog < 6 else -1

                # Use Joint Ids to index corresponding min/max jog speed.
                min_speed = PAROL6_ROBOT.Joint_min_jog_speed[joint_id]
                max_speed = PAROL6_ROBOT.Joint_max_jog_speed[joint_id]

                speed_val = direction * int(np.interp(self.Jog_control[0], [0, 100], [min_speed, max_speed]))

                msg = move_joints(cmd_data, robot_data, [joint_id], [speed_val])
                self.shared_string.value = msg.encode()[:120]  # Limit length for shared string.

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
                msg = cartesian_jog(
                    cmd_data=cmd_data,
                    robot_data=robot_data,
                    dx=dx, dy=dy, dz=dz,
                    rx=rx, ry=ry, rz=rz,
                    frame="WRF" if self.Jog_control[2] == 1 else "TRF",
                    speed_pct=self.Jog_control[0],
                    interval_s=INTERVAL_S
                )

                self.shared_string.value = msg.encode()[:120]  # Cap string length
                # Calculate every joint speed using var and q1

                # commanded position = robot position
                # if real send to real
                # if sim send to sim 
                # if both send to both 
                #print(result_joint_jog)

            elif self.Buttons[0] == 1: # HOME COMMAND 0x100
                
                cmd_data.command.value = 100
                self.Buttons[0] = 0
                self.shared_string.value = b'Log: Robot homing'

            elif self.Buttons[1] == 1: # ENABLE COMMAND 0x101
                
                cmd_data.command.value = 101 
                self.Buttons[1] = 0
                self.shared_string.value = b'Log: Robot enable'

            elif self.Buttons[2] == 1 or InOut_in[4] == 0: # DISABLE COMMAND 0x102
                Robot_mode = "STOP"
                self.Buttons[7] = 0 # program execution button
                
                cmd_data.command.value = 102
                self.Buttons[2] = 0
                self.shared_string.value = b'Log: Robot disable; button or estop'

            elif self.Buttons[3] == 1: # CLEAR ERROR COMMAND 0x103
                
                cmd_data.command.value = 103
                self.Buttons[3] = 0
                self.shared_string.value = b'Log: Error clear'

            elif self.Buttons[6] == 1: # For testing accel motions?
                
                cmd_data.command.value = 69
            # Program execution
            ######################################################

            else: # If nothing else is done send dummy data 0x255
                Robot_mode = "Dummy"
                dummy_data(
                    cmd_data.position,
                cmd_data.speed,
                cmd_data.command,Position_in)

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

    
    def plan(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
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

            log = cartesian_jog(
                cmd_data=cmd_data,
                robot_data=robot_data,
                dx=dx, dy=dy, dz=dz,
                rx=rx, ry=ry, rz=rz,
                frame="WRF" if self.jog_control[2] == 1 else "TRF",
                speed_pct=self.jog_control[0],
                interval_s=INTERVAL_S
            )
            self.shared_string.value = log.encode()[:120]

        else:
            self.dx, self.dy, self.dz = (0, 0, 0)
            dummy_data(cmd_data.position,
                    cmd_data.speed,
                    cmd_data.command,
                    robot_data.position)

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
        
