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
from tools.data_process_tools import *

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
def Send_data(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons):
    timer = Timer(INTERVAL_S, warnings=False, precise=True)
    cnt = 0

    while timer.elapsed_time < 110000:

        if ser.is_open == True:
            logging.debug("Task 1 alive")
            logging.debug("Data that PC will send to the robot is: ")
            #s = Pack_data_test() 
            # This function packs data that we will send to the robot
            s = Pack_data(Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out)
            
            # Make sure if sending calib to gripper to send it only once
            if(Gripper_data_out[4] == 1 or Gripper_data_out[4] == 2):
                Gripper_data_out[4] = 0

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

            
            # JOINT JOG (regular speed control) 0x123 # -1 is value if nothing is pressed
            if result_joint_jog != -1 and Buttons[2] == 0 and InOut_in[4] == 1: 
                Robot_mode = "Joint jog"
                Command_out.value = 123 
                # Set speed for all other joints to 0
                for i in range(6):
                    Speed_out[i] = 0
                    # ako je position in veći ili jednak nekom od limita disable tu stranu tipki
                # Set speed for the clicked joint
                if(result_joint_jog in [0,1,2,3,4,5]):
                    if Position_in[result_joint_jog] >= PAROL6_ROBOT.Joint_limits_steps[result_joint_jog][1]:
                        shared_string.value = b'Error: Robot jog -> Position out of range' 
                    else:
                        Speed_out[result_joint_jog ] =  int(np.interp(Jog_control[0],[0,100],[PAROL6_ROBOT.Joint_min_jog_speed[result_joint_jog],PAROL6_ROBOT.Joint_max_jog_speed[result_joint_jog]]))
                        arr = bytes(str(result_joint_jog + 1), 'utf-8')
                        shared_string.value = b'Log: Joint  ' + arr +   b'  jog  ' 
                else:
                    if Position_in[result_joint_jog-6] <= PAROL6_ROBOT.Joint_limits_steps[result_joint_jog-6][0]:
                        shared_string.value = b'Error: Robot jog -> Position out of range'
                    else:  
                        Speed_out[result_joint_jog - 6] =  int(-1 * np.interp(Jog_control[0],[0,100],[PAROL6_ROBOT.Joint_min_jog_speed[result_joint_jog-6],PAROL6_ROBOT.Joint_max_jog_speed[result_joint_jog-6]]))
                        arr = bytes(str(result_joint_jog - 6 + 1), 'utf-8')
                        shared_string.value = b'Log: Joint  ' + arr +   b'  jog  ' 

           
            ######################################################
            ######################################################
            # CART JOG (regular speed control but for multiple joints) 0x123 # -1 is value if nothing is pressed
            elif result_cart_jog != -1 and Buttons[2] == 0 and InOut_in[4] == 1: #

                Command_out.value = 123
                # Set speed for all other joints to 0
                for i in range(6):
                    Speed_out[i] = 0
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
                        Speed_out[i] = int(PAROL6_ROBOT.SPEED_RAD2STEP(temp_var[i],i))
                        prev_speed[i] = Speed_out[i]
                    else:
                        shared_string.value = b'Error: Inverse kinematics error '
                        #Command_out.value = 102
                        #Speed_out[i] = 0
                    # If any joint passed its position limit, disable robot


                if Robot_mode != "Cartesian jog":
                    for i in range(6):
                        if abs(Speed_out[i]) >= 300000:
                            Speed_out[i] = int(Speed_out[i] / 10000)
                            arr = bytes(str(Speed_out[i]), 'utf-8')
                            arr2 = bytes(str(i+1),'utf-8')
                            shared_string.value = b'Error: Joint  ' + arr2  +   b'  speed error in cart mode  '+ arr


                else:
                    # If any joint starts moving faster than allowed DISABLE ROBOT
                    for i in range(6):
                        if abs(Speed_out[i]) >= 300000:
                            Command_out.value = 102
                            arr = bytes(str(Speed_out[i]), 'utf-8')
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
                Command_out.value = 100
                Buttons[0] = 0
                shared_string.value = b'Log: Robot homing'

            
            elif Buttons[1] == 1: # ENABLE COMMAND 0x101
                Command_out.value = 101 
                Buttons[1] = 0
                shared_string.value = b'Log: Robot enable'

            elif Buttons[2] == 1 or InOut_in[4] == 0: # DISABLE COMMAND 0x102
                Robot_mode = "STOP"
                Buttons[7] = 0 # program execution button
                Command_out.value = 102
                Buttons[2] = 0
                shared_string.value = b'Log: Robot disable; button or estop'

            elif Buttons[3] == 1: # CLEAR ERROR COMMAND 0x103
                Command_out.value = 103
                Buttons[3] = 0
                shared_string.value = b'Log: Error clear'

            elif Buttons[6] == 1: # For testing accel motions?
                Command_out.value = 69
            

            # Program execution
            ######################################################
            ######################################################

            ######################################################
            ######################################################
            else: # If nothing else is done send dummy data 0x255
                Robot_mode = "Dummy"
                dummy_data(Position_out,Speed_out,Command_out,Position_in)

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

def extract_content_from_command(command):
    match = re.search(r'\((.*?)\)', command)
    if match:
        return match.group(1)
    else:
        return None

# Task that receives data and saves to the multi proc array
def Receive_data(shared_string,Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,General_data):
    while 1:

        # 🐍 PYTHON Serial Explanation
        # inWaiting() tells how many bytes are currently available in the serial input buffer.
        # Since we continuously send serial data, there will usually be something in the buffer.
        # That's why we often call ser.read(ser.inWaiting()) to read all available bytes at once.
        # If the data is arriving too slowly, the buffer might be empty (inWaiting == 0),
        # and the program simply skips reading and continues to the next loop.
        # It checks again on the next loop iteration and reads if data has arrived.

        # Reference:
        # https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial.in_waiting
        # https://stackoverflow.com/questions/17553543/pyserial-non-blocking-read-loop

        # 🛠️ ARDUINO Serial Explanation
        # Serial.available() tells how many bytes are available in the buffer.
        # Serial.read() only reads ONE byte at a time, so you need to use a while(Serial.available()) loop
        # and manually accumulate the bytes into a buffer.
        # When a specific character (like '\n') is received, or when the buffer reaches a certain length,
        # you parse the buffer and check its validity.
        # Just like in Python, if data is too slow to arrive, Serial.available() may return 0,
        # even if you're inside a while loop — it simply skips until new data comes in.
        # If you were using an if() instead of while(), you'd only read one byte per loop,
        # and if the rest of your code is slow, you might receive serial data too slowly or miss bytes.

        # Reference:
        # https://forum.arduino.cc/t/sending-command-over-serial-64-bytes-128-bytes/121598/3
        # https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
        # http://www.gammon.com.au/serial
        try:
            Get_data(Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in)
            #Get_data_old()
        except:
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
        #Get_data_old()
        #print("Task 2 alive")
        #time.sleep(2)


# Treba mi bytes format za slanje, nije baš user readable je pretvori iz hex u ascii
# ako trebam gledati vrijednosti koristi hex() funkciju
# 我需要发送用的 bytes 格式，这不是面向用户的可读格式（user readable），
# 它是十六进制（hex）转换成 ASCII 的格式。
# 如果我要查看数值，就用 hex() 函数。
# I need the bytes format for sending; it's not really user-readable — it's hex to ASCII.
# If I need to view the values, I use the hex() function.

# Dummy test task
# Best used to show data that we get from the robot and data we get from GUI
def Monitor_system(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons):
    while(1):
        # Construct dictionaries for printing.
        packet_info = {
            "start_bytes_list":      [0xff,0xff,0xff],
            "start_bytes_bytes":     bytes([0xff,0xff,0xff]),
            "Test Split Two Bytes":       Split_2_bitfield(123),
            "Test Split Three Bytes":  Split_2_3_bytes(-235005),
            "Test Fusing Three Bytes":            Fuse_3_bytes(Split_2_3_bytes(-235005)),
            "Test Fusing Two Bytes":    Fuse_bitfield_2_bytearray(Split_2_bitfield(123)),
        }
        
        # The test is not clever enough to self validate, enhance this in future.
        # test_list = [10]*50
        # elements = list(range(60))
        # Unpack_data_test(elements)
        # print("$$$$$$$$$$$$$$$$")
        # Pack_data_test()
    	
        robot_data = {
            "Position":     Position_in[:],
            "Speed":        Speed_in[:],
            "Homed":        Homed_in[:],
            "I/O status":   InOut_in[:],
            "Temp error":   Temperature_error_in[:],
            "Pos error":    Position_error_in[:],
            "Timeout err":  Timeout_error.value,
            "Δt raw":       Timing_data_in.value,
            "Δt (ms)":      f"{Timing_data_in.value*1.42222222e-6:.3f}",
            "XTR byte":     XTR_data.value,
            "Grip ID":      Gripper_data_in[0],
            "Grip pos":     Gripper_data_in[1],
            "Grip spd":     Gripper_data_in[2],
            "Grip cur":     Gripper_data_in[3],
            "Grip stat":    Gripper_data_in[4],
            "Obj detect":   Gripper_data_in[5],
        }

        commanded_data = {
            "I/O out":      InOut_out[:],
            "Command":      Command_out.value,
            "Speed out":    Speed_out[:],
        }

        gui_data = {
            "Joint jog":    list(Joint_jog_buttons),
            "Cart jog":     list(Cart_jog_buttons),
            "Home btn":     Buttons[0],
            "Enable btn":   Buttons[1],
            "Disable btn":  Buttons[2],
            "Clear err":    Buttons[3],
            "Real/Sim":     f"{Buttons[4]}/{Buttons[5]}",
            "Speed slider": Jog_control[0],
            "WRF/TRF":      Jog_control[2],
            "Demo app":     Buttons[6],
            "Exec state":   Buttons[7],
            "Park btn":     Buttons[8],
            "Shared str":   shared_string.value.decode().strip(),
        }

        # Print the data at once.
        nice_print_sections({
            #"Packet Info":     packet_info,
            "Robot Data":      robot_data,
            "Commanded Data":  commanded_data,
            "GUI Data":        gui_data,
        })

        time.sleep(3)


# Just read data and print it
def Get_data_old():
    while (ser.inWaiting() > 0):
        data_str = ser.read(ser.inWaiting()) #.decode('utf-8') 
        print(data_str)
        print("\\+\\") 
        time.sleep(0.01)    



    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52 #1
    Position = [255,255,255,255,255,255]  #18
    Speed = [255,255,255,255,255,255]  #18
    Command = 123 #1 
    Affected_joint = [1,1,1,1,1,1,1,1] #1
    InOut = [0,0,0,0,0,0,0,0] #1
    Timeout = 247 #1
    Gripper_data = [-222,-223,-224,225,226,123]  #9
    CRC_byte = 228 #1
    # End bytes = 2


    test_list = []
    #print(test_list)

    #x = bytes(start_bytes)
    test_list.append((start_bytes))
    
    test_list.append(bytes([len]))

    # Position data
    for i in range(6):
        position_split = Split_2_3_bytes(Position[i])
        test_list.append(position_split[1:4])

    # Speed data
    for i in range(6):
        speed_split = Split_2_3_bytes(Speed[i])
        test_list.append(speed_split[1:4])

    # Command data
    test_list.append(bytes([Command]))

    # Affected joint data
    Affected_list = Fuse_bitfield_2_bytearray(Affected_joint)
    test_list.append(Affected_list)

    # Inputs outputs data
    InOut_list = Fuse_bitfield_2_bytearray(InOut)
    test_list.append(InOut_list)

    # Timeout data
    test_list.append(bytes([Timeout]))

    # Gripper position
    Gripper_position = Split_2_3_bytes(Gripper_data[0])
    test_list.append(Gripper_position[2:4])

    # Gripper speed
    Gripper_speed = Split_2_3_bytes(Gripper_data[1])
    test_list.append(Gripper_speed[2:4])

    # Gripper current
    Gripper_current = Split_2_3_bytes(Gripper_data[2])
    test_list.append(Gripper_current[2:4])  

    # Gripper command
    test_list.append(bytes([Gripper_data[3]]))
    # Gripper mode
    test_list.append(bytes([Gripper_data[4]]))
    # Gripper ID
    test_list.append(bytes([Gripper_data[5]]))
 
    # CRC byte
    test_list.append(bytes([CRC_byte]))

    # END bytes
    test_list.append((end_bytes))
    
    #print(test_list)
    return test_list

def Get_data(Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in):
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
                    Unpack_data(data_buffer, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
                    XTR_data,Gripper_data_in)
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
