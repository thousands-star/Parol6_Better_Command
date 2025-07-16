import time
from abc import ABC, abstractmethod
from tools.shared_struct import RobotInputData, RobotOutputData, check_elements
from tools.PAROL6_ROBOT import Joint_min_jog_speed, Joint_max_jog_speed
import numpy as np
from typing import List
from multiprocessing import Array
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
from tools.log_tools import nice_print_sections

INTERVAL_S = 0.01
Robot_mode = "Dummy"

def dummy_data(Position_out,Speed_out,Command_out,Position_in):
    Command_out.value = 255
    for i in range(6):
        Position_out[i] = Position_in[i]
        Speed_out[i] = 0

class Reactor(ABC):
    """
    抽象化的决策模块，用于根据 robot_state 和上一次命令生成下一帧命令。
    所有 GUI/AI/脚本控制器都应继承并实现 plan 方法。
    """
    def __init__(self, robot_output: RobotOutputData, robot_input: RobotInputData):
        self.robot_data = robot_input
        self.cmd_data = robot_output
        self.prev_speed = [0,0,0,0,0,0]
        self.counter = 0

    def giveCommand(self):
        """
        每帧调用的接口，用于统一调用时序逻辑。
        返回值：新一帧的 RobotOutputData
        """
        self.plan()
        if(self.cmd_data.gripper_data[4] == 1 or self.cmd_data.gripper_data[4] == 2):
            self.cmd_data.gripper_data[4] = 0

        # if self.counter < 500:
        #     self.counter = self.counter + 1
        # else:
        #     Initprint = {
        #         "robot_data": self.robot_data.to_dict(),
        #         "cmd_data": self.cmd_data.to_dict()
        #     }

        #     nice_print_sections(Initprint)
        #     self.counter = 0

        return self.cmd_data.pack()
    
    
 

    @abstractmethod
    def plan(self) -> List[bytes]:
        return self.robot_data.pack()


class GUIReactor(Reactor):
    def __init__(self, 
                 robotOutputData: RobotOutputData, 
                 robotInputData: RobotInputData,
                shared_string:       Array,                  
                Joint_jog_buttons:   Array,             # multiprocessing.Array("i", [..])
                Cart_jog_buttons:    Array,             # multiprocessing.Array("i", [..])
                Jog_control:         Array,             # multiprocessing.Array("i", [..])
                Buttons:             Array,             # multiprocessing.Array("i", [..])
                 ):
        
        super().__init__(robot_output=robotOutputData, robot_input=robotInputData)
        self.shared_string = shared_string
        self.Joint_jog_buttons = Joint_jog_buttons
        self.Cart_jog_buttons = Cart_jog_buttons
        self.Jog_control = Jog_control
        self.Buttons = Buttons


    def plan(self):
            # Check if any of jog buttons is pressed
            result_joint_jog = check_elements(list(self.Joint_jog_buttons))
            result_cart_jog = check_elements(list(self.Cart_jog_buttons))

            ######################################################
            ######################################################
            InOut_in = self.robot_data.inout
            Position_in = self.robot_data.position

            
            # JOINT JOG (regular speed control) 0x123 # -1 is value if nothing is pressed
            if result_joint_jog != -1 and self.Buttons[2] == 0 and InOut_in[4] == 1: 
                Robot_mode = "Joint jog"
                self.cmd_data.command.value = 123 
                # Set speed for all other joints to 0
                for i in range(6):
                    self.cmd_data.speed[i] = 0
                    # ako je position in veći ili jednak nekom od limita disable tu stranu tipki
                # Set speed for the clicked joint
                if(result_joint_jog in [0,1,2,3,4,5]):
                    if Position_in[result_joint_jog] >= PAROL6_ROBOT.Joint_limits_steps[result_joint_jog][1]:
                        self.shared_string.value = b'Error: Robot jog -> Position out of range' 
                    else:
                        self.cmd_data.speed[result_joint_jog ] =  int(np.interp(self.Jog_control[0],[0,100],[PAROL6_ROBOT.Joint_min_jog_speed[result_joint_jog],PAROL6_ROBOT.Joint_max_jog_speed[result_joint_jog]]))
                        arr = bytes(str(result_joint_jog + 1), 'utf-8')
                        self.shared_string.value = b'Log: Joint  ' + arr +   b'  jog  ' 
                else:
                    if Position_in[result_joint_jog-6] <= PAROL6_ROBOT.Joint_limits_steps[result_joint_jog-6][0]:
                        self.shared_string.value = b'Error: Robot jog -> Position out of range'
                    else:  
                        self.cmd_data.speed[result_joint_jog - 6] =  int(-1 * np.interp(self.Jog_control[0],[0,100],[PAROL6_ROBOT.Joint_min_jog_speed[result_joint_jog-6],PAROL6_ROBOT.Joint_max_jog_speed[result_joint_jog-6]]))
                        arr = bytes(str(result_joint_jog - 6 + 1), 'utf-8')
                        self.shared_string.value = b'Log: Joint  ' + arr +   b'  jog  ' 

            ######################################################
            ######################################################
            # CART JOG (regular speed control but for multiple joints) 0x123 # -1 is value if nothing is pressed
            elif result_cart_jog != -1 and self.Buttons[2] == 0 and InOut_in[4] == 1: #

                self.cmd_data.command.value = 123
                # Set speed for all other joints to 0
                for i in range(6):
                    self.cmd_data.speed[i] = 0
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
                        self.cmd_data.speed[i] = int(PAROL6_ROBOT.SPEED_RAD2STEP(temp_var[i],i))
                        self.prev_speed[i] = self.cmd_data.speed[i]
                    else:
                        self.shared_string.value = b'Error: Inverse kinematics error '
                        #Command_out.value = 102
                        #Speed_out[i] = 0
                    # If any joint passed its position limit, disable robot


                if Robot_mode != "Cartesian jog":
                    for i in range(6):
                        if abs(self.cmd_data.speed[i]) >= 300000:
                            self.cmd_data.speed[i] = int(self.cmd_data.speed[i] / 10000)
                            arr = bytes(str(self.cmd_data.speed[i]), 'utf-8')
                            arr2 = bytes(str(i+1),'utf-8')
                            self.shared_string.value = b'Error: Joint  ' + arr2  +   b'  speed error in cart mode  '+ arr


                else:
                    # If any joint starts moving faster than allowed DISABLE ROBOT
                    for i in range(6):
                        if abs(self.cmd_data.speed[i]) >= 300000:
                            self.cmd_data.command.value = 102
                            arr = bytes(str(self.cmd_data.speed[i]), 'utf-8')
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
                self.cmd_data.command.value = 100
                self.Buttons[0] = 0
                self.shared_string.value = b'Log: Robot homing'

            elif self.Buttons[1] == 1: # ENABLE COMMAND 0x101
                self.cmd_data.command.value = 101 
                self.Buttons[1] = 0
                self.shared_string.value = b'Log: Robot enable'

            elif self.Buttons[2] == 1 or InOut_in[4] == 0: # DISABLE COMMAND 0x102
                Robot_mode = "STOP"
                self.Buttons[7] = 0 # program execution button
                self.cmd_data.command.value = 102
                self.Buttons[2] = 0
                self.shared_string.value = b'Log: Robot disable; button or estop'

            elif self.Buttons[3] == 1: # CLEAR ERROR COMMAND 0x103
                self.cmd_data.command.value = 103
                self.Buttons[3] = 0
                self.shared_string.value = b'Log: Error clear'

            elif self.Buttons[6] == 1: # For testing accel motions?
                self.cmd_data.command.value = 69
            # Program execution
            ######################################################
            ######################################################

            ######################################################
            ######################################################
            else: # If nothing else is done send dummy data 0x255
                Robot_mode = "Dummy"
                dummy_data(self.cmd_data.position,self.cmd_data.speed,self.cmd_data.command,Position_in)

        
