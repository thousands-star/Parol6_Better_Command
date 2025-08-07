from oclock import Timer, loop, interactiveloop
import time, random
import time
import roboticstoolbox as rp
import struct
import logging
import tools.PAROL6_ROBOT as PAROL6_ROBOT 
import threading
import numpy as np
from spatialmath import *
from tools.init_tools import get_my_os, get_image_path
from tools.log_tools import nice_print_sections
from tools.shared_struct import RobotInputData, RobotOutputData
from multiprocessing import Value, Array, Semaphore
from Commander import Commander

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

logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

logging.disable(logging.DEBUG)

ser = None
LOGINTERVAL = 3
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
prev_mode = None

Robot_mode = "Dummy"
# Task for sending data every x ms and performing all calculations, kinematics GUI control logic...
def Send_data(
    General_data:        Array,              # multiprocessing.Array("i", [port, baud])
    cmd_data:            RobotOutputData,
    Robot_mode:             Value,
    stop_event:           threading.Event,
    sync_sema:             Semaphore,
    ready_sema:            Semaphore
) -> None:
    
    timer = Timer(INTERVAL_S, warnings=False, precise=True)
    cnt = 0
    prev_mode = Robot_mode

    while timer.elapsed_time < 110000:
        if stop_event.is_set():
            break

        if ser.is_open == True:
            logging.debug("Task 1 alive")
            logging.debug("Data that PC will send to the robot is: ")

            # This function packs data that we will send to the robot
            sync_sema.release()     
            ready_sema.acquire()    # 没准备好就卡住
            s = cmd_data.pack()
            

            logging.debug(s)
            logging.debug("END of data sent to the ROBOT")
            len_ = len(s)
            try:
                for i in range(len_):
                    ser.write(s[i])
            except:
                logging.debug("NO SERIAL TASK1")
                    # This function packs data that we will send to the robot
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
    logging.info("[Serial Sender] Sender thread was closed properly.")


def Receive_data(general_data: list, robot_data: RobotInputData, exit_event:threading.Event):
    """
    Continuously read packets into `shared` via get_data(shared).
    On any read error, attempt to reconnect serial using general_data[0].
    """
    while not exit_event.is_set():
        try:
            # blocks until one full packet is processed into `shared`
            Get_data(robot_data)
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
    logging.info("[Serial Sender] Receiving Thread was closed properly.")


# Treba mi bytes format za slanje, nije baš user readable je pretvori iz hex u ascii
# ako trebam gledati vrijednosti koristi hex() funkciju
# 我需要发送用的 bytes 格式，这不是面向用户的可读格式（user readable），
# 它是十六进制（hex）转换成 ASCII 的格式。
# 如果我要查看数值，就用 hex() 函数。
# I need the bytes format for sending; it's not really user-readable — it's hex to ASCII.
# If I need to view the values, I use the hex() function.

# Dummy test task
# Best used to show data that we get from the robot and data we get from GUI

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