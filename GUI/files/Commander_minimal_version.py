# A stripped down commander software code
# You can write your own functions here easily and communicate
# with other  scripts on your PC or local network via UDP
# In this example you can perform some actions via keyboard:
# * Pressing h will home the robot
# * Pressing e will enable robot
# * If you press estop robot will stop and you need to enable it by pressing e

from roboticstoolbox import DHRobot, RevoluteDH, ERobot, ELink, ETS
from tools.data_process_tools import Split_2_3_bytes, Split_2_bitfield, Pack_data, Pack_data_test, Unpack_data, Unpack_data_test, Fuse_2_bytes, Fuse_3_bytes, Fuse_bitfield_2_bytearray, check_elements
from math import pi, sin, cos
from tools.init_tools import get_image_path, get_my_os
import numpy as np
from oclock import Timer, loop, interactiveloop
import time
import socket
import select
import serial
import platform
import os
import logging
import struct
import keyboard

# Set interval
INTERVAL_S = 0.01
prev_time = 0

logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)
logging.disable(logging.DEBUG)


my_os = platform.system()
if my_os == "Windows": 
    STARTING_PORT = 3 # COM3
str_port = ''

if my_os == "Windows":
    try:
        str_port = 'COM' + str(STARTING_PORT)
        ser = serial.Serial(port=str_port, baudrate=3000000, timeout=0)
    except:
        ser = serial.Serial()



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

#######################################################################################
#######################################################################################
Position_out = [1,11,111,1111,11111,10]
Speed_out = [2,21,22,23,24,25]
Command_out = 255
Affected_joint_out = [1,1,1,1,1,1,1,1]
InOut_out = [0,0,0,0,0,0,0,0]
Timeout_out = 0
#Positon,speed,current,command,mode,ID
Gripper_data_out = [1,1,1,1,0,0]
#######################################################################################
#######################################################################################
# Data sent from robot to PC
Position_in = [31,32,33,34,35,36]
Speed_in = [41,42,43,44,45,46]
Homed_in = [1,1,1,1,1,1,1,1]
InOut_in = [1,1,1,1,1,1,1,1]
Temperature_error_in = [1,1,1,1,1,1,1,1]
Position_error_in = [1,1,1,1,1,1,1,1]
Timeout_error = 0
# how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
Timing_data_in = [0]
XTR_data =   0

#ID,Position,speed,current,status,obj_detection
Gripper_data_in = [1,1,1,1,1,1] 




#Setup IP address and Simulator port
ip = "127.0.0.1" #Loopback address
port = 5001
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))
print(f'Start listening to {ip}:{port}')

def home_robot_key():
    global Command_out
    Command_out = 100
    print("Home robot")

def start_position():
    print("Go to start position")

def enable_robot():
    global Command_out
    Command_out = 101
    print("Enable robot")


keyboard.add_hotkey('h', home_robot_key)
keyboard.add_hotkey('j', start_position)
keyboard.add_hotkey('e', enable_robot)

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


# Set interval
timer = Timer(interval=INTERVAL_S, warnings=False, precise=True)
while timer.elapsed_time < 1100000:
    
    time1 = time.perf_counter()
    print(f"Loop time is: {time1-prev_time}")
    print(f"Command out is {Command_out}")
    print(f"inout is: {InOut_in}")
    #print(f"timing data in is: {Timing_data_in[0]* 1.4222222e-6 }")

    prev_time = time1

    if ser.is_open == True:


        s = Pack_data(Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out)
        len_ = len(s)
        try:
            for i in range(len_):
                ser.write(s[i])
        except:
            logging.debug("NO SERIAL TASK1")

        Get_data(Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
            XTR_data,Gripper_data_in)
        

        # Safety stuff
        if(Command_out == 100):
            Command_out = 255

        if(InOut_in[4] == 0):
            Command_out = 102
            for i in range(6):
                Position_out[i] = Position_in[i]
                Speed_out[i] = 0



    else:
        try:
            if my_os == 'Linux':
                com_port = '/dev/ttyACM' + str(STARTING_PORT)
            elif my_os == 'Windows':
                com_port = 'COM' + str(STARTING_PORT)
                
            print(com_port)
            ser.port = com_port
            ser.baudrate = 3000000
            ser.close()
            time.sleep(0.5)
            ser.open()
            time.sleep(0.5)
        except:
            time.sleep(0.5)


    ############################################################
    # Get data from UDP
    data = None
    while True:
        try:
            # Check if the socket is ready to read
            ready_to_read, _, _ = select.select([sock], [], [], 0)  # Timeout of 0 second, no blocking read instantly
            #print(ready_to_read)
            # Check if there's data available to read
            if sock in ready_to_read:
                # Receive data from the socket
                data, addr = sock.recvfrom(1024)  # data needs to be decoded                    
            else:
                #print(f"No sock in ready")
                break
        except KeyboardInterrupt:
            # Handle keyboard interrupt
            break
        except Exception as e:
            # Handle other exceptions
            print(f"Error: {e}")
            break
    ############################################################


    # Process the last received packet after exiting the loop
    if data:
        print(data)
        received_data = data.decode()

    timer.checkpt()
