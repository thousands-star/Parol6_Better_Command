import time
import multiprocessing
import threading
import logging
import sys

import SIMULATOR_Robot
from tools.init_tools import init_serial, get_image_path,get_my_os
import Serial_sender_latest
import GUI_PAROL_latest
from tools.shared_struct import RobotInputData, RobotOutputData
from Reactor import GUIReactor,FollowTagReactor
from Commander import Commander, Mode
from commander_loop import commander_loop, Monitor_system

from robot_api import start_flask_api
from FlaskKiller import FlaskShutdownThread


logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

logging.disable(logging.DEBUG)

ser, STARTING_PORT = init_serial()
my_os = get_my_os()
image_path = get_image_path()

def SIMULATOR_process(Position_out,Robot_data:RobotInputData,Position_Sim,Buttons, stop_event:threading.Event):
    SIMULATOR_Robot.GUI(Position_out,Robot_data,Position_Sim,Buttons, stop_event)

def Arm_communication(General_data,robot_data:RobotInputData, cmd_data:RobotOutputData, Robot_mode, stop_event:threading.Event, sync_sema:multiprocessing.Semaphore, ready_sema:multiprocessing.Semaphore): 

    global ser, my_os
    Serial_sender_latest.ser = ser
    Serial_sender_latest.my_os = my_os
    Serial_sender_latest.LOGINTERVAL = 3

    t1 = threading.Thread(target = Serial_sender_latest.Send_data, args = (General_data,cmd_data,Robot_mode,stop_event,sync_sema, ready_sema))
    
    t2 = threading.Thread(target = Serial_sender_latest.Receive_data, args = (General_data,robot_data,stop_event))
    
    # t3 = threading.Thread(target = Serial_sender_latest.Monitor_system,args = (Commander,Robot_mode,stop_event))

    t1.start()
    t2.start()
    # t3.start()


    logging.info("[Serial Sender] Serial Sender process was shutted down gracefully.")

def GUI_process(shared_string,Command_data:RobotOutputData,
         Robot_data:RobotInputData,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,display_q, robot_mode, stop_event):

        logging.info("[GUI] GUI process was initiated.")

        GUI_PAROL_latest.GUI(shared_string,Command_data,Robot_data,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,display_q, robot_mode, stop_event)
        
        logging.info("[GUI] process was shutted down properly.")

def Camera_process(frame_q, display_q, detected_tags, stop_event):
    """
    Start 3 threads:
    - camera_capture_thread: captures images and puts them into frame_q
    - tag_detector_thread: detects AprilTags from frame_q
    - overlay_thread: draws overlays and puts into display_q
    """
    from tools.Camera.CameraBase import LogitechCamera
    from Vision import camera_capture_thread, tag_detector_thread, overlay_thread
    
    tag_lock = threading.Lock()

    cam = LogitechCamera(source=1, width=640, height=480, fps=30)

    t1 = threading.Thread(target=camera_capture_thread, args=(cam, frame_q,stop_event), daemon=True)
    t2 = threading.Thread(target=tag_detector_thread, args=(frame_q,detected_tags, tag_lock, stop_event), daemon=True)
    t3 = threading.Thread(target=overlay_thread, args=(frame_q, display_q, tag_lock, stop_event), daemon=True)

    logging.info("[Camera Process] Starting threads (capture, detect, overlay)...")
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()

    frame_q.close()
    display_q.close()
    detected_tags.close()
    frame_q.cancel_join_thread()
    display_q.cancel_join_thread()
    detected_tags.cancel_join_thread()

    logging.info("[Camera Process] All threads exited, cleaning up and terminating process.")


if __name__ == '__main__':

    print("running")
    time.sleep(0.01) 

    try:
        ser.close()
    except:
        None

    # Data sent by the PC to the robot
    Command_data = RobotOutputData()
    Command_data.initialize(
        position_init=[1, 11, 111, 1111, 11111, 10],
        speed_init=[2, 21, 22, 23, 24, 25],
        affected_joint_init=[1, 1, 1, 1, 1, 1, 1, 1],
        inout_init=[0, 0, 0, 0, 0, 0, 0, 0],
        gripper_init=[1, 1, 1, 1, 0, 0],
        command_init=0,
        timeout_init=0
    )

    # Data sent from robot to PC. This is the initialization that would work according to the protocol. 
    Robot_data = RobotInputData()
    Robot_data.initialize(
        position_init=[31, 32, 33, 34, 35, 36],
        speed_init=[41, 42, 43, 44, 45, 46],
        homed_init=[1] * 8,
        inout_init=[1] * 8,
        temperature_error_init=[1] * 8,
        position_error_init=[1] * 8,
        gripper_init=[1] * 6,
    )



    # GUI control data
    Homed_out = multiprocessing.Array("i",[1,1,1,1,1,1], lock=False) 

    #General robot vars
    Robot_mode =   multiprocessing.Value('i',Mode.GUI)

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

    detected_tags = multiprocessing.Queue(maxsize=2)

    guireactor = GUIReactor(
                           shared_string=shared_string, 
                           Joint_jog_buttons=Joint_jog_buttons, 
                           Cart_jog_buttons=Cart_jog_buttons,
                           Jog_control=Jog_control,
                           Buttons=Buttons
                           )
    
    followtagreactor = FollowTagReactor(detected_tags=detected_tags, jog_control=Jog_control,shared_string=shared_string)
    
    plugins = {Mode.GUI: guireactor, Mode.FOLLOW_TAG: followtagreactor}
    
    commander = Commander(cmd_data=Command_data, robot_data= Robot_data, plugins=plugins, current_mode=Robot_mode)
    
    # Image Queue for camera
    frame_q = multiprocessing.Queue(maxsize=1)
    display_q = multiprocessing.Queue(maxsize=1)
    stop_event = multiprocessing.Event()
    sync_event = multiprocessing.Event()

    sync_sema = multiprocessing.Semaphore(0)
    ready_sema = multiprocessing.Semaphore(0)
    


 
    process1 = multiprocessing.Process(target=Arm_communication,args=[General_data,Robot_data,Command_data, Robot_mode, stop_event,sync_sema,ready_sema])
    
    process2 = multiprocessing.Process(target=GUI_process,args=[shared_string,Command_data,Robot_data,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,display_q, Robot_mode, stop_event])

    process3 = multiprocessing.Process(target=Camera_process, args=[frame_q, display_q, detected_tags, stop_event])

    # Due to unknown reason, it is hard to implement safe close event into process4, We would just disable it since it is not significant in our usage.
    # process4 = multiprocessing.Process(target=SIMULATOR_process,args =[Command_data,Robot_data,Position_Sim,Buttons, stop_event])

    threading.Thread(target=commander_loop, args=(commander, stop_event, sync_sema, ready_sema), daemon=True).start()
    threading.Thread(target=Monitor_system, args=(commander, Robot_mode, stop_event), daemon=True).start()
    threading.Thread(target=start_flask_api, args=(commander,shared_string), daemon=True).start()
    flask_killer = FlaskShutdownThread(stop_event)
    flask_killer.start()


    processes = [process1, process2]
    # 启动所有进程（加点延时防 race）
    for p in processes:
        p.start()
        print(f"[Startup] Started process: {p.name}")
        time.sleep(1)

    # 等待所有进程退出
    for p in processes:
        p.join()

    print("All processes ended. See you again!")
    sys.exit(0)

    logging.info("See you again!")
    sys.exit(0)