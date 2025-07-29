import time
from abc import ABC, abstractmethod
from tools.shared_struct import RobotInputData, RobotOutputData, check_elements
import numpy as np
from typing import List, Dict, Any
from multiprocessing import Array
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
from tools.StatelessAction import move_joints
from tools.log_tools import nice_print_sections
import queue
from statistics import median           
from typing import List, Tuple, Optional

INTERVAL_S = 0.01
Robot_mode = "Dummy"

def dummy_data(Position_out,Speed_out,Command_out,Position_in):
    Command_out.value = 255
    for i in range(6):
        Position_out[i] = Position_in[i]
        Speed_out[i] = 0

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
                cmd_data.command.value = 123
                # Set speed for all other joints to 0
                for i in range(6):
                    cmd_data.speed[i] = 0
                # if moving in positive direction

                q1 = np.array([PAROL6_ROBOT.STEPS2RADS(Position_in[0],0),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[1],1),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[2],2),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[3],3),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[4],4),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[5],5),])
                T = PAROL6_ROBOT.robot.fkine(q1)

                temp_var = float(np.interp(self.Jog_control[0],[0,100],[PAROL6_ROBOT.Cartesian_linear_velocity_min_JOG,PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG]))
                temp_var_angular = float(np.interp(self.Jog_control[0],[0,100],[PAROL6_ROBOT.Cartesian_angular_velocity_min,PAROL6_ROBOT.Cartesian_angular_velocity_max]))

                speed_temp = temp_var # Speed is 20mm/s = 0.02m/s
                speed_temp_angular = temp_var_angular # Speed is DEG/s

                delta_s = speed_temp * INTERVAL_S # displacement in meters
                delta_s_angular = speed_temp_angular * INTERVAL_S # displacement in degrees
                

                # WRF jogging
                if self.Jog_control[2] == 1: # WRF jog
                    if result_cart_jog in [1,3,4,10,7,8]: # For positive directions 1,3,4.
                        if result_cart_jog == 4: # Z+ direction
                            T.t[2] = T.t[2] + delta_s  # Add to the Z+ direction in WRF
                            self.shared_string.value = b'Log: Cartesian WRF Z+ move'
                        elif result_cart_jog == 3: # Y+ direction
                            T.t[1] = T.t[1] + delta_s  # Add to the Y+ direction in WRF
                            self.shared_string.value = b'Log: Cartesian WRF Y+ move'
                        elif result_cart_jog == 1: # X+ direction
                            T.t[0] = T.t[0] + delta_s  # Add to the X+ direction in WRF
                            self.shared_string.value = b'Log: Cartesian WRF X+ move'

                        elif result_cart_jog == 10: # Rotation in Z+ direction
                            None
                        elif result_cart_jog == 8: # Rotation in Y+ direction
                            None
                        elif result_cart_jog == 7: # Rotation in X+ direction
                            None

                    # if moving in negative direction
                    else:
                        if result_cart_jog == 5: # Z- direction
                            T.t[2] = T.t[2] - delta_s  # Add to the Z- direction in WRF
                            self.shared_string.value = b'Log: Cartesian WRF Z- move'
                        elif result_cart_jog == 2: # Y- direction
                            T.t[1] = T.t[1] - delta_s  # Add to the Y- direction in WRF
                            self.shared_string.value = b'Log: Cartesian WRF Y- move'
                        elif result_cart_jog == 0: # X- direction
                            T.t[0] = T.t[0] - delta_s  # Add to the X- direction in WRF
                            self.shared_string.value = b'Log: Cartesian WRF X- move'

                        elif result_cart_jog == 11: # Rotation in Z- direction
                            None
                        elif result_cart_jog == 9: # Rotation in Y- direction
                            None
                        elif result_cart_jog == 6: # Rotation in X- direction
                            None

                # TRF jogging
                else: # TRF jog
                    if result_cart_jog in [1,3,4,10,7,8]: # For positive directions 1,3,4.
                        if result_cart_jog == 4: # Z+ direction
                            x1 = [0,0,delta_s] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            self.shared_string.value = b'Log: Cartesian TRF Z+ move'
                        elif result_cart_jog == 3: # Y+ direction
                            x1 = [0,delta_s,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            self.shared_string.value = b'Log: Cartesian TRF Y+ move'
                        elif result_cart_jog == 1: # X+ direction
                            x1 = [delta_s,0,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            self.shared_string.value = b'Log: Cartesian TRF X+ move'

                        elif result_cart_jog == 10:  # Rotation in Z+ direction
                            T2  = T * T.Rz(delta_s_angular,'deg')
                            T = T2
                            self.shared_string.value = b'Log: TRF Z+ Rotation '
                        elif result_cart_jog == 8:   # Rotation in Y+ direction
                            T2 = T * T.Ry(delta_s_angular,'deg')
                            T = T2
                            self.shared_string.value = b'Log: TRF Y+ Rotation '
                        elif result_cart_jog == 7:   # Rotation in x+ direction
                            T2 = T * T.Rx(delta_s_angular,'deg')
                            T  = T2
                            self.shared_string.value = b'Log: TRF X+ Rotation '


                    # if moving in negative direction
                    else:
                        if result_cart_jog == 5: # Z- direction
                            x1 = [0,0,-delta_s] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            self.shared_string.value = b'Log: Cartesian TRF Z- move'
                        elif result_cart_jog == 2: # Y- direction
                            x1 = [0,-delta_s,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            self.shared_string.value = b'Log: Cartesian TRF Y- move'
                        elif result_cart_jog == 0: # X- direction
                            x1 = [-delta_s,0,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            self.shared_string.value = b'Log: Cartesian TRF X- move'

                        elif result_cart_jog == 11:  # Rotation in Z- direction
                            T2  = T * T.Rz(-delta_s_angular,'deg')
                            T = T2
                            self.shared_string.value = b'Log: TRF Z- Rotation '
                        elif result_cart_jog == 9:  # Rotation in Y- direction
                            T2  = T * T.Ry(-delta_s_angular,'deg')
                            T = T2
                            self.shared_string.value = b'Log: TRF Y- Rotation '
                        elif result_cart_jog == 6:  # Rotation in X- direction
                            T2  = T * T.Rx(-delta_s_angular,'deg')
                            T = T2
                            self.shared_string.value = b'Log: TRF X- Rotation '
                        

                var = PAROL6_ROBOT.robot.ikine_LMS(T,q0 = q1, ilimit = 6) # Get joint angles

                temp_var = [0,0,0,0,0,0]
                for i in range(6):

                    temp_var[i] = ((var[0][i] - q1[i]) / INTERVAL_S)
                    #print(temp_var)

                    # If solver gives error DISABLE ROBOT
                    if var.success:
                        
                        cmd_data.speed[i] = int(PAROL6_ROBOT.SPEED_RAD2STEP(temp_var[i],i))
                        self.prev_speed[i] = cmd_data.speed[i]
                    else:
                        self.shared_string.value = b'Error: Inverse kinematics error '
                        #Command_out.value = 102
                        #Speed_out[i] = 0
                    # If any joint passed its position limit, disable robot


                if Robot_mode != "Cartesian jog":
                    for i in range(6):
                        if abs(
                            cmd_data.speed[i]) >= 300000:
                            
                            cmd_data.speed[i] = int(
                                cmd_data.speed[i] / 10000)
                            arr = bytes(str(
                                cmd_data.speed[i]), 'utf-8')
                            arr2 = bytes(str(i+1),'utf-8')
                            self.shared_string.value = b'Error: Joint  ' + arr2  +   b'  speed error in cart mode  '+ arr


                else:
                    # If any joint starts moving faster than allowed DISABLE ROBOT
                    for i in range(6):
                        if abs(
                            cmd_data.speed[i]) >= 300000:
                            
                            cmd_data.command.value = 102
                            arr = bytes(str(
                                cmd_data.speed[i]), 'utf-8')
                            arr2 = bytes(str(i+1),'utf-8')
                            self.shared_string.value = b'Error: Joint  ' + arr2  +   b'  speed error in cart mode  '+ arr
                Robot_mode = "Cartesian jog"
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
            ######################################################

            ######################################################
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
            self.dz = self.dz - 0.3

            if abs(self.dz) > 0.1:
                self.dz = 0

            if abs(self.dx) < 0.05:
                self.dx = 0

            if abs(self.dy) < 0.05:
                self.dy = 0

            robotdy = self.dx
            robotdz = self.dy
            
            cartesian_jog_step(cmd_data,
                        robot_data,
                        0, robotdy, -robotdz,
                        self.jog_control,
                        self.shared_string)
        else:
            self.dx, self.dy, self.dz = (0,0,0)
            dummy_data(cmd_data.position,
                    cmd_data.speed,
                    cmd_data.command,robot_data.position)

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
        
