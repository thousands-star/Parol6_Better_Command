import time
import multiprocessing
import threading
from multiprocessing import Manager

import SIMULATOR_Robot
from tools.init_tools import init_serial, get_image_path,get_my_os
import Serial_sender_good_latest
import GUI_PAROL_latest

ser, STARTING_PORT = init_serial()
my_os = get_my_os()
image_path = get_image_path()

def SIMULATOR_process(Position_out,Position_in,Position_Sim,Buttons):
    SIMULATOR_Robot.GUI(Position_out,Position_in,Position_Sim,Buttons)

def Arm_communication(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,): 

    global ser, my_os
    Serial_sender_good_latest.ser = ser
    Serial_sender_good_latest.my_os = my_os

    t1 = threading.Thread(target = Serial_sender_good_latest.Send_data, args = (shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons))
    
    t2 = threading.Thread(target = Serial_sender_good_latest.Receive_data, args = (shared_string, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,General_data,))
    
    t3 = threading.Thread(target = Serial_sender_good_latest.Monitor_system,args = ( shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons))

    t1.start()
    t2.start()
    t3.start()

def GUI_process(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,display_q):

        GUI_PAROL_latest.GUI(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,display_q)

def Camera_process(frame_q, display_q, stop_event):
    """
    Start 3 threads:
    - camera_capture_thread: captures images and puts them into frame_q
    - tag_detector_thread: detects AprilTags from frame_q
    - overlay_thread: draws overlays and puts into display_q
    """
    from tools.Camera.CameraBase import LogitechCamera
    from Vision import camera_capture_thread, tag_detector_thread, overlay_thread
    
    tag_list = []
    tag_lock = threading.Lock()

    cam = LogitechCamera(source=1, width=640, height=480, fps=30)

    t1 = threading.Thread(target=camera_capture_thread, args=(cam, frame_q,stop_event), daemon=True)
    t2 = threading.Thread(target=tag_detector_thread, args=(frame_q,tag_list, tag_lock, stop_event), daemon=True)
    t3 = threading.Thread(target=overlay_thread, args=(frame_q, display_q, tag_list, tag_lock, stop_event), daemon=True)

    print("[Camera Process] Starting threads (capture, detect, overlay)...")
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()

     

if __name__ == '__main__':

    print("running")
    time.sleep(0.01) 

    try:
        ser.close()
    except:
        None

    # Data sent by the PC to the robot
    Position_out = multiprocessing.Array("i",[1,11,111,1111,11111,10], lock=False) 

    Speed_out = multiprocessing.Array("i",[2,21,22,23,24,25], lock=True)


    Command_out = multiprocessing.Value('i',0) 
    Affected_joint_out = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    InOut_out = multiprocessing.Array("i",[0,0,0,0,0,0,0,0], lock=False) #IN1,IN2,OUT1,OUT2,ESTOP
    Timeout_out = multiprocessing.Value('i',0) 
    #Positon,speed,current,command,mode,ID
    Gripper_data_out = multiprocessing.Array("i",[1,1,1,1,0,0], lock=False)

    # Data sent from robot to PC
    Position_in = multiprocessing.Array("i",[31,32,33,34,35,36], lock=False) 
    Speed_in = multiprocessing.Array("i",[41,42,43,44,45,46], lock=False) 
    Homed_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    InOut_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) #IN1,IN2,OUT1,OUT2,ESTOP
    Temperature_error_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    Position_error_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    Timeout_error = multiprocessing.Value('i',0) 
    # how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
    Timing_data_in = multiprocessing.Value('i',0) 
    XTR_data =   multiprocessing.Value('i',0)

    #ID,Position,speed,current,status,obj_detection
    Gripper_data_in = multiprocessing.Array("i",[1,1,1,1,1,1], lock=False)  

    # GUI control data
    Homed_out = multiprocessing.Array("i",[1,1,1,1,1,1], lock=False) 

    #General robot vars
    Robot_GUI_mode =   multiprocessing.Value('i',0)

    # Robot jogging vars
    Joint_jog_buttons = multiprocessing.Array("i",[0,0,0,0,0,0,0,0,0,0,0,0], lock=False) 
    Cart_jog_buttons = multiprocessing.Array("i",[0,0,0,0,0,0,0,0,0,0,0,0], lock=False)

    # Speed slider, acc slider, WRF/TRF 
    Jog_control = multiprocessing.Array("i",[0,0,0,0], lock=False) 

    # COM PORT, BAUD RATE, 
    General_data =  multiprocessing.Array("i",[STARTING_PORT,3000000], lock=False) 

    # Home,Enable,Disable,Clear error,Real_robot,Sim_robot, demo_app, program execution,
    Buttons =  multiprocessing.Array("i",[0,0,0,0,1,1,0,0,0], lock=False) 

    # Positions for robot simulator
    Position_Sim =  multiprocessing.Array("i",[0,0,0,0,0,0], lock=False) 

    shared_string = multiprocessing.Array('c', b' ' * 100)  # Create a character array of size 100
    
    # Image Queue for camera
    frame_q = multiprocessing.Queue(maxsize=1)
    display_q = multiprocessing.Queue(maxsize=1)
    stop_event = multiprocessing.Event()


    # Process
    process1 = multiprocessing.Process(target=Arm_communication,args=[shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,])
    
    process2 = multiprocessing.Process(target=GUI_process,args=[shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,display_q])
    
    process3 = multiprocessing.Process(target=SIMULATOR_process,args =[Position_out,Position_in,Position_Sim,Buttons])

    process4 = multiprocessing.Process(target=Camera_process, args=[frame_q, display_q, stop_event])
    


    process1.start()
    time.sleep(1)
    process2.start()
    time.sleep(1)
    process3.start()
    time.sleep(1)
    process4.start()

    process1.join()
    process2.join()
    process3.join()
    process4.join()

    process1.terminate()
    process2.terminate()
    process3.terminate()
    process4.terminate()