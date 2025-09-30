import time
import multiprocessing
import threading
import logging
import sys
import os
from datetime import datetime

import SIMULATOR_Robot
from tools.init_tools import init_serial, get_image_path,get_my_os
import Serial_sender_latest
import GUI_PAROL_latest
from tools.shared_struct import RobotInputData, RobotOutputData
from Reactor import GUIReactor,FollowTagReactor, IBVSReactor
from Commander import Commander, Mode
from commander_loop import commander_loop, Monitor_system

from robot_api import start_flask_api
from FlaskKiller import FlaskShutdownThread

from Centroid_IBVS import CentroidIBVSReactor


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

def Eye_in_hand_process(frame_q, display_q, detected_tags, uv_queue, stop_event):
    """
    Start 4 threads:
    - camera_capture_thread: frames -> proc_q & raw_display_q
    - tag_detector_thread: detections -> Vision.detected_tags + detected_tags queue
    - uv_publisher_thread: detected_tags -> uv_multi_q  (list of up to 2)
    - overlay_thread: raw_display_q + uv_multi_q -> display_q (GUI)
    """
    from tools.Camera.CameraBase import LogitechCamera, ESP32Camera
    import numpy as np
    from Vision import camera_capture_thread, tag_detector_thread, overlay_thread, uv_publisher_thread

    tag_lock = threading.Lock()
    param_dir = os.path.join(os.path.dirname(__file__), "tools", "Camera", "param")
    K = np.loadtxt(os.path.join(param_dir, "camera_intrinsic_matrix.csv"), delimiter=',').astype(np.float32)
    dist = np.loadtxt(os.path.join(param_dir, "distortion_coeffs.csv"), delimiter=',').astype(np.float32)

    cam = ESP32Camera(url="http://172.20.10.2", mode="stream",K=K, dist=dist)
    import queue
    # local queues inside this process
    raw_display_q = multiprocessing.Queue(maxsize=2)   # camera -> overlay
    uv_multi_q    = uv_queue   # publisher(list) -> overlay & adapter

    # threads
    t1 = threading.Thread(target=camera_capture_thread,
                          args=(cam, frame_q, raw_display_q, stop_event),
                          daemon=True)

    # Use your detected_tags queue here; Vision.tag_detector_thread supports queue snapshots
    t2 = threading.Thread(target=tag_detector_thread,
                          args=(frame_q, detected_tags,cam, tag_lock, stop_event),
                          daemon=True)

    # NEW: publish up to two tags as list
    t3 = threading.Thread(target=uv_publisher_thread,
                          args=(tag_lock, uv_multi_q, stop_event),
                          kwargs=dict(preferred_tag_ids=None, publish_hz=0.5, cam=cam, max_tags=2),
                          daemon=True)

    # overlay consumes raw_display_q and uv_multi_q, outputs to the display_q you passed to GUI
    t4 = threading.Thread(target=overlay_thread,
                          args=(raw_display_q, display_q, tag_lock, stop_event),
                          daemon=True)

    logging.info("[Camera Process] Starting threads (capture, detect, publish, adapt, overlay)...")
    for t in (t1, t2, t3, t4):
        t.start()
    for t in (t1, t2, t3, t4):
        t.join()

    # cleanup
    for q in (frame_q, display_q, detected_tags):
        try:
            q.close()
            q.cancel_join_thread()
        except Exception:
            pass

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
    
    
    uv_queue = multiprocessing.Queue(maxsize=1)

    DESIRED_UV_8 = [
    # Tag 6
    (193,60), (193,41), (174,40), (174,58),
    # Tag 9
    (121,56), (141,56), (142,38), (123,38),
    ]

    DESIRED_UV_BENCH = [
    # Tag 2
    (256,120), (255,99), (236,99), (237,120),
    # Tag 5
    (85,122), (84,102), (66,102), (66,122),
    ]

    logfile = os.path.join("logs", f"ibvs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl")


    ibvs_reactor = IBVSReactor(
        tags_q        = uv_queue,
        jog_control   = Jog_control,
        shared_string = shared_string,        
        target_ids    = [6, 9], #6,9
        desired_uv    = DESIRED_UV_8,
        # 控制参数（先稳一点）
        lam           = 0.6,
        mu            = 1e-3,
        max_step_m    = 0.001,
        max_rot_rad   = 0.01,
        use_depth     = True,           # 若 pose_t[2] 可用，建议 True
        depth_est     = 0.50,
        # 单位对齐：你的 cartesian_jog 用的是 unit='deg'，所以这里用度
        angles_in_rad = False,
        # 坐标映射（默认 Cam X->X, Cam Y->-Y, Cam Z->Z）
        # P_override   = P_swap,        # 若需要旧映射，把上面 P_swap 打开并传入
        # 终止条件
        stop_on_converge = True,
        converge_px_rms  = 15.0,
        converge_frames  = 50,
        timeout_s        = None,
        stagnation_eps   = 1e-6,
        stagnation_frames= 30,
        # 轴掩码（排障好用，1=启用；先全开）
        axis_mask_lin = (1,1,0),
        axis_mask_ang = (-1,1,1),
        rvec_mask=(1,1,0),
        # JSON Lines 日志（便于仿真/复盘）
        log_path   = logfile,
        log_every  = 1
    )

    centroid_reactor = CentroidIBVSReactor(
    detected_tags_queue = detected_tags,   # multiprocessing.Queue
    jog_control = Jog_control,             # multiprocessing.Array (speed slider)
    shared_string = shared_string,         # multiprocessing.Array('c', ...) - status log
    desired_uv = None,                     # None -> will use principal point cx,cy from K if present
    depth_m = 0.30,                        # 估计深度（m），视你的相机-目标距离调
    lam = 0.9,                             # 比例增益
    deadband_px = 6.0,                     # 像素死区
    max_step_m = 0.01,                     # 每 tick 最大位移 (m) 保守起见设小
    invert_x = False,
    invert_y = True,
    frame = "TRF",
    speed_pct_default = 30.0,
    interval_s = 0.01
    )
   

    plugins = {Mode.GUI: guireactor, Mode.IBVS: ibvs_reactor}
    
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

    process3 = multiprocessing.Process(target=Eye_in_hand_process, args=[frame_q, display_q, detected_tags, uv_queue, stop_event])

    # Due to unknown reason, it is hard to implement safe close event into process4, We would just disable it since it is not significant in our usage.
    # process4 = multiprocessing.Process(target=SIMULATOR_process,args =[Command_data,Robot_data,Position_Sim,Buttons, stop_event])

    threading.Thread(target=commander_loop, args=(commander, stop_event, sync_sema, ready_sema), daemon=True).start()
    threading.Thread(target=Monitor_system, args=(commander, Robot_mode, stop_event), daemon=True).start()
    threading.Thread(target=start_flask_api, args=(commander,shared_string), daemon=True).start()
    flask_killer = FlaskShutdownThread(stop_event)
    flask_killer.start()


    processes = [process1, process2,process3]
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