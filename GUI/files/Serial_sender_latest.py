from oclock import Timer, loop, interactiveloop
import time, random
import time
import roboticstoolbox as rp
import struct
import logging
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
import numpy as np
from spatialmath import *
from tools.init_tools import get_my_os, get_image_path
from tools.log_tools import nice_print_sections
from tools.shared_struct import RobotInputData,RobotOutputData,check_elements
from multiprocessing import Value, Array

import re
import math
from roboticstoolbox import trapezoidal
from roboticstoolbox import quintic
from spatialmath.base.argcheck import (
    isvector,
    getvector,
    # assertmatrix,
    getvector,
    isscalar,
)

# Finds out where the program and images are stored
my_os = get_my_os()
Image_path = get_image_path()

print("run this")
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

logging.disable(logging.DEBUG)

ser = None
#ser.open()


# Data for testing
#######################################################################################

data_len_output = [0x05]
data_len_output = bytes(data_len_output)

test_data =  [0x9,0x4,0x5] 
test_data = bytes(test_data)
#######################################################################################


# data for input string (Data that is being sent by the robot)
#######################################################################################
#######################################################################################
input_byte = 0 # Here save incoming bytes from serial

start_cond1_byte = bytes([0xff])
start_cond2_byte = bytes([0xff])
start_cond3_byte = bytes([0xff])

end_cond1_byte = bytes([0x01])
end_cond2_byte = bytes([0x02])

start_cond1 = 0 #Flag if start_cond1_byte is received
start_cond2 = 0 #Flag if start_cond2_byte is received
start_cond3 = 0 #Flag if start_cond3_byte is received

good_start = 0 #Flag if we got all 3 start condition bytes
data_len = 0 #Length of the data after -3 start condition bytes and length byte, so -4 bytes

data_buffer = [None]*255 #Here save all data after data length byte
data_counter = 0 #Data counter for incoming bytes; compared to data length to see if we have correct length
#######################################################################################
#######################################################################################

prev_positions = [0,0,0,0,0,0]

# Set interval
INTERVAL_S = 0.01

robot_pose = [0,0,0,0,0,0] #np.array([0,0,0,0,0,0])
prev_speed = [0,0,0,0,0,0]
Tt = SE3
interval_test = 0

# Program execution variables
Program_length = 0
Program_step = 0

Robot_mode = "Dummy"
# Task for sending data every x ms and performing all calculations, kinematics GUI control logic...
def Send_data(
    shared_string:       Array,             # multiprocessing.Array('c', …)
    Command_data:         RobotOutputData,
    Robot_data:           RobotInputData,    # your dataclass for all incoming arrays/values
    Joint_jog_buttons:   Array,             # multiprocessing.Array("i", [..])
    Cart_jog_buttons:    Array,             # multiprocessing.Array("i", [..])
    Jog_control:         Array,             # multiprocessing.Array("i", [..])
    General_data:        Array,             # multiprocessing.Array("i", [port, baud])
    Buttons:             Array,             # multiprocessing.Array("i", [..])
) -> None:
    timer = Timer(INTERVAL_S, warnings=False, precise=True)
    cnt = 0

    while timer.elapsed_time < 110000:

        if ser.is_open == True:
            logging.debug("Task 1 alive")
            logging.debug("Data that PC will send to the robot is: ")

            # This function packs data that we will send to the robot
            s = Command_data.pack()
            
            # Make sure if sending calib to gripper to send it only once
            if(Command_data.gripper_data[4] == 1 or Command_data.gripper_data[4] == 2):
                Command_data.gripper_data[4] = 0

            logging.debug(s)
            logging.debug("END of data sent to the ROBOT")
            len_ = len(s)
            try:
                for i in range(len_):
                    ser.write(s[i])
            except:
                logging.debug("NO SERIAL TASK1")
                    # This function packs data that we will send to the robot
    

            # Check if any of jog buttons is pressed
            result_joint_jog = check_elements(list(Joint_jog_buttons))
            result_cart_jog = check_elements(list(Cart_jog_buttons))

            ######################################################
            ######################################################
            InOut_in = Robot_data.inout
            Position_in = Robot_data.position

            
            # JOINT JOG (regular speed control) 0x123 # -1 is value if nothing is pressed
            if result_joint_jog != -1 and Buttons[2] == 0 and InOut_in[4] == 1: 
                Robot_mode = "Joint jog"
                Command_data.command.value = 123 
                # Set speed for all other joints to 0
                for i in range(6):
                    Command_data.speed[i] = 0
                    # ako je position in veći ili jednak nekom od limita disable tu stranu tipki
                # Set speed for the clicked joint
                if(result_joint_jog in [0,1,2,3,4,5]):
                    if Position_in[result_joint_jog] >= PAROL6_ROBOT.Joint_limits_steps[result_joint_jog][1]:
                        shared_string.value = b'Error: Robot jog -> Position out of range' 
                    else:
                        Command_data.speed[result_joint_jog ] =  int(np.interp(Jog_control[0],[0,100],[PAROL6_ROBOT.Joint_min_jog_speed[result_joint_jog],PAROL6_ROBOT.Joint_max_jog_speed[result_joint_jog]]))
                        arr = bytes(str(result_joint_jog + 1), 'utf-8')
                        shared_string.value = b'Log: Joint  ' + arr +   b'  jog  ' 
                else:
                    if Position_in[result_joint_jog-6] <= PAROL6_ROBOT.Joint_limits_steps[result_joint_jog-6][0]:
                        shared_string.value = b'Error: Robot jog -> Position out of range'
                    else:  
                        Command_data.speed[result_joint_jog - 6] =  int(-1 * np.interp(Jog_control[0],[0,100],[PAROL6_ROBOT.Joint_min_jog_speed[result_joint_jog-6],PAROL6_ROBOT.Joint_max_jog_speed[result_joint_jog-6]]))
                        arr = bytes(str(result_joint_jog - 6 + 1), 'utf-8')
                        shared_string.value = b'Log: Joint  ' + arr +   b'  jog  ' 

           
            ######################################################
            ######################################################
            # CART JOG (regular speed control but for multiple joints) 0x123 # -1 is value if nothing is pressed
            elif result_cart_jog != -1 and Buttons[2] == 0 and InOut_in[4] == 1: #

                Command_data.command.value = 123
                # Set speed for all other joints to 0
                for i in range(6):
                    Command_data.speed[i] = 0
                # if moving in positive direction
                q1 = np.array([PAROL6_ROBOT.STEPS2RADS(Position_in[0],0),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[1],1),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[2],2),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[3],3),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[4],4),
                                PAROL6_ROBOT.STEPS2RADS(Position_in[5],5),])
                T = PAROL6_ROBOT.robot.fkine(q1)

                temp_var = float(np.interp(Jog_control[0],[0,100],[PAROL6_ROBOT.Cartesian_linear_velocity_min_JOG,PAROL6_ROBOT.Cartesian_linear_velocity_max_JOG]))
                temp_var_angular = float(np.interp(Jog_control[0],[0,100],[PAROL6_ROBOT.Cartesian_angular_velocity_min,PAROL6_ROBOT.Cartesian_angular_velocity_max]))

                speed_temp = temp_var # Speed is 20mm/s = 0.02m/s
                speed_temp_angular = temp_var_angular # Speed is DEG/s

                delta_s = speed_temp * INTERVAL_S # displacement in meters
                delta_s_angular = speed_temp_angular * INTERVAL_S # displacement in degrees
                

                # WRF jogging
                if Jog_control[2] == 1: # WRF jog
                    if result_cart_jog in [1,3,4,10,7,8]: # For positive directions 1,3,4.
                        if result_cart_jog == 4: # Z+ direction
                            T.t[2] = T.t[2] + delta_s  # Add to the Z+ direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Z+ move'
                        elif result_cart_jog == 3: # Y+ direction
                            T.t[1] = T.t[1] + delta_s  # Add to the Y+ direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Y+ move'
                        elif result_cart_jog == 1: # X+ direction
                            T.t[0] = T.t[0] + delta_s  # Add to the X+ direction in WRF
                            shared_string.value = b'Log: Cartesian WRF X+ move'

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
                            shared_string.value = b'Log: Cartesian WRF Z- move'
                        elif result_cart_jog == 2: # Y- direction
                            T.t[1] = T.t[1] - delta_s  # Add to the Y- direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Y- move'
                        elif result_cart_jog == 0: # X- direction
                            T.t[0] = T.t[0] - delta_s  # Add to the X- direction in WRF
                            shared_string.value = b'Log: Cartesian WRF X- move'

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
                            shared_string.value = b'Log: Cartesian TRF Z+ move'
                        elif result_cart_jog == 3: # Y+ direction
                            x1 = [0,delta_s,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF Y+ move'
                        elif result_cart_jog == 1: # X+ direction
                            x1 = [delta_s,0,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF X+ move'

                        elif result_cart_jog == 10:  # Rotation in Z+ direction
                            T2  = T * T.Rz(delta_s_angular,'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Z+ Rotation '
                        elif result_cart_jog == 8:   # Rotation in Y+ direction
                            T2 = T * T.Ry(delta_s_angular,'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Y+ Rotation '
                        elif result_cart_jog == 7:   # Rotation in x+ direction
                            T2 = T * T.Rx(delta_s_angular,'deg')
                            T  = T2
                            shared_string.value = b'Log: TRF X+ Rotation '


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
                            shared_string.value = b'Log: Cartesian TRF Z- move'
                        elif result_cart_jog == 2: # Y- direction
                            x1 = [0,-delta_s,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF Y- move'
                        elif result_cart_jog == 0: # X- direction
                            x1 = [-delta_s,0,0] 
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF X- move'

                        elif result_cart_jog == 11:  # Rotation in Z- direction
                            T2  = T * T.Rz(-delta_s_angular,'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Z- Rotation '
                        elif result_cart_jog == 9:  # Rotation in Y- direction
                            T2  = T * T.Ry(-delta_s_angular,'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Y- Rotation '
                        elif result_cart_jog == 6:  # Rotation in X- direction
                            T2  = T * T.Rx(-delta_s_angular,'deg')
                            T = T2
                            shared_string.value = b'Log: TRF X- Rotation '
                        

                var = PAROL6_ROBOT.robot.ikine_LMS(T,q0 = q1, ilimit = 6) # Get joint angles

                temp_var = [0,0,0,0,0,0]
                for i in range(6):

                    temp_var[i] = ((var[0][i] - q1[i]) / INTERVAL_S)
                    #print(temp_var)

                    # If solver gives error DISABLE ROBOT
                    if var.success:
                        Command_data.speed[i] = int(PAROL6_ROBOT.SPEED_RAD2STEP(temp_var[i],i))
                        prev_speed[i] = Command_data.speed[i]
                    else:
                        shared_string.value = b'Error: Inverse kinematics error '
                        #Command_out.value = 102
                        #Speed_out[i] = 0
                    # If any joint passed its position limit, disable robot


                if Robot_mode != "Cartesian jog":
                    for i in range(6):
                        if abs(Command_data.speed[i]) >= 300000:
                            Command_data.speed[i] = int(Command_data.speed[i] / 10000)
                            arr = bytes(str(Command_data.speed[i]), 'utf-8')
                            arr2 = bytes(str(i+1),'utf-8')
                            shared_string.value = b'Error: Joint  ' + arr2  +   b'  speed error in cart mode  '+ arr


                else:
                    # If any joint starts moving faster than allowed DISABLE ROBOT
                    for i in range(6):
                        if abs(Command_data.speed[i]) >= 300000:
                            Command_data.command.value = 102
                            arr = bytes(str(Command_data.speed[i]), 'utf-8')
                            arr2 = bytes(str(i+1),'utf-8')
                            shared_string.value = b'Error: Joint  ' + arr2  +   b'  speed error in cart mode  '+ arr
                Robot_mode = "Cartesian jog"
                # Calculate every joint speed using var and q1



                # commanded position = robot position
                # if real send to real
                # if sim send to sim 
                # if both send to both 
                #print(result_joint_jog)

          

            elif Buttons[0] == 1: # HOME COMMAND 0x100
                Command_data.command.value = 100
                Buttons[0] = 0
                shared_string.value = b'Log: Robot homing'

            
            elif Buttons[1] == 1: # ENABLE COMMAND 0x101
                Command_data.command.value = 101 
                Buttons[1] = 0
                shared_string.value = b'Log: Robot enable'

            elif Buttons[2] == 1 or InOut_in[4] == 0: # DISABLE COMMAND 0x102
                Robot_mode = "STOP"
                Buttons[7] = 0 # program execution button
                Command_data.command.value = 102
                Buttons[2] = 0
                shared_string.value = b'Log: Robot disable; button or estop'

            elif Buttons[3] == 1: # CLEAR ERROR COMMAND 0x103
                Command_data.command.value = 103
                Buttons[3] = 0
                shared_string.value = b'Log: Error clear'

            elif Buttons[6] == 1: # For testing accel motions?
                Command_data.command.value = 69
            

            # Program execution
            ######################################################
            ######################################################

            ######################################################
            ######################################################
            else: # If nothing else is done send dummy data 0x255
                Robot_mode = "Dummy"
                dummy_data(Command_data.position,Command_data.speed,Command_data.command,Position_in)

            # Provjere 
            # Svaka move funkciaj će imati svoje provjere!
            # Tu će samo biti zadnja provjera koja gleda brzine i ako su pre velike stavlja ih na nula!
            time2 = time.perf_counter()
    
        else:
            try:
                if my_os == 'Linux':
                    com_port = '/dev/ttyACM' + str(General_data[0])
                elif my_os == 'Windows':
                    com_port = 'COM' + str(General_data[0])
                    
                print(com_port)
                ser.port = com_port
                ser.baudrate = 3000000
                ser.close()
                time.sleep(0.5)
                ser.open()
                time.sleep(0.5)
            except:
                time.sleep(0.5)
                logging.debug("no serial available, reconnecting!")   

        timer.checkpt()


def dummy_data(Position_out,Speed_out,Command_out,Position_in):
    Command_out.value = 255
    for i in range(6):
        Position_out[i] = Position_in[i]
        Speed_out[i] = 0




def Receive_data(Robot_data: RobotInputData, general_data: list):
    """
    Continuously read packets into `shared` via get_data(shared).
    On any read error, attempt to reconnect serial using general_data[0].
    """
    while True:
        try:
            # blocks until one full packet is processed into `shared`
            Get_data(Robot_data)
        except Exception as read_err:
            logging.debug("Read error: %s", read_err)
            # attempt to reconnect
            try:
                idx = general_data[0]
                if my_os == 'Linux':
                    com_port = f'/dev/ttyACM{idx}'
                else:
                    com_port = f'COM{idx}'
                logging.info("Reconnecting on port %s...", com_port)
                ser.port = com_port
                ser.close()
                time.sleep(0.5)
                ser.open()
                time.sleep(0.5)
            except Exception as conn_err:
                logging.debug("Reconnect failed: %s", conn_err)
                time.sleep(0.5)


# Treba mi bytes format za slanje, nije baš user readable je pretvori iz hex u ascii
# ako trebam gledati vrijednosti koristi hex() funkciju
# 我需要发送用的 bytes 格式，这不是面向用户的可读格式（user readable），
# 它是十六进制（hex）转换成 ASCII 的格式。
# 如果我要查看数值，就用 hex() 函数。
# I need the bytes format for sending; it's not really user-readable — it's hex to ASCII.
# If I need to view the values, I use the hex() function.

# Dummy test task
# Best used to show data that we get from the robot and data we get from GUI
def Monitor_system(
    shared_string:       Array,            # e.g. Array('c', …)
    Command_data:            RobotOutputData,
    Robot_data:              RobotInputData,   # all incoming data wrapped here
    Joint_jog_buttons:   Array,             # multiprocessing.Array("i", [..])
    Cart_jog_buttons:    Array,             # multiprocessing.Array("i", [..])
    Jog_control:         Array,             # multiprocessing.Array("i", [..])
    General_data:        Array,             # multiprocessing.Array("i", [port, baud])
    Buttons:             Array,             # multiprocessing.Array("i", [..])
) -> None:
    while True:
        # 1) Print robot inputs
        robot_data = Robot_data.to_dict()

        # 2) Print commanded outputs
        command_data = Command_data.to_dict()

        # 3) Print GUI state
        gui = {
            "Joint jog":  list(Joint_jog_buttons),
            "Cart jog":   list(Cart_jog_buttons),
            "Home":       Buttons[0],
            "Enable":     Buttons[1],
            "Disable":    Buttons[2],
            "Clear err":  Buttons[3],
            "Real/Sim":   f"{Buttons[4]}/{Buttons[5]}",
            "Speed sl":   Jog_control[0],
            "WRF/TRF":    Jog_control[2],
            "Demo":       Buttons[6],
            "Execute":    Buttons[7],
            "Park":       Buttons[8],
            "Log msg":    shared_string.value.decode().strip(),
        }

        # 4) Print everything in one go
        nice_print_sections({
            "Robot Data":      robot_data,
            "Commanded Data":  command_data,
            "GUI State":       gui,
        })

        time.sleep(3)


def Get_data(shared: RobotInputData):
    """
    Read from serial until a full, well-framed packet arrives,
    then unpack its payload into `shared`.
    """
    global input_byte 

    global start_cond1_byte 
    global start_cond2_byte 
    global start_cond3_byte 

    global end_cond1_byte 
    global end_cond2_byte 

    global start_cond1 
    global start_cond2 
    global start_cond3 

    global good_start 
    global data_len 

    global data_buffer 
    global data_counter

    while (ser.inWaiting() > 0):
        input_byte = ser.read()

        #UNCOMMENT THIS TO GET ALL DATA FROM THE ROBOT PRINTED
        #print(input_byte) 

        # When data len is received start is good and after that put all data in receive buffer
        # Data len is ALL data after it; that includes input buffer, end bytes and CRC
        if (good_start != 1):
            # All start bytes are good and next byte is data len
            if (start_cond1 == 1 and start_cond2 == 1 and start_cond3 == 1):
                good_start = 1
                data_len = input_byte
                data_len = struct.unpack('B', data_len)[0]
                logging.debug("data len we got from robot packet= ")
                logging.debug(input_byte)
                logging.debug("good start for DATA that we received at PC")
            # Third start byte is good
            if (input_byte == start_cond3_byte and start_cond2 == 1 and start_cond1 == 1):
                start_cond3 = 1
                #print("good cond 3 PC")
            #Third start byte is bad, reset all flags
            elif (start_cond2 == 1 and start_cond1 == 1):
                #print("bad cond 3 PC")
                start_cond1 = 0
                start_cond2 = 0
            # Second start byte is good
            if (input_byte == start_cond2_byte and start_cond1 == 1):
                start_cond2 = 1
                #print("good cond 2 PC ")
            #Second start byte is bad, reset all flags   
            elif (start_cond1 == 1):
                #print("Bad cond 2 PC")
                start_cond1 = 0
            # First start byte is good
            if (input_byte == start_cond1_byte):
                start_cond1 = 1
                #print("good cond 1 PC")
        else:
            # Here data goes after good  start
            data_buffer[data_counter] = input_byte
            if (data_counter == data_len - 1):

                logging.debug("Data len PC")
                logging.debug(data_len)
                logging.debug("End bytes are:")
                logging.debug(data_buffer[data_len -1])
                logging.debug(data_buffer[data_len -2])

                # Here if last 2 bytes are end condition bytes we process the data 
                if (data_buffer[data_len -1] == end_cond2_byte and data_buffer[data_len - 2] == end_cond1_byte):

                    logging.debug("GOOD END CONDITION PC")
                    logging.debug("I UNPACKED RAW DATA RECEIVED FROM THE ROBOT")
                    shared.unpack(data_buffer)
                    logging.debug("DATA UNPACK FINISHED")
                    # ako su dobri izračunaj crc
                    # if crc dobar raspakiraj podatke
                    # ako je dobar paket je dobar i spremi ga u nove variable!
                
                # Print every byte
                #print("podaci u data bufferu su:")
                #for i in range(data_len):
                    #print(data_buffer[i])

                good_start = 0
                start_cond1 = 0
                start_cond3 = 0
                start_cond2 = 0
                data_len = 0
                data_counter = 0
            else:
                data_counter = data_counter + 1