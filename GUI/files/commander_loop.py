# commander_loop.py

import time
from multiprocessing import Event, Value, Semaphore
import threading
from Commander import Commander
from tools.log_tools import nice_print_sections
import logging
from tools.shared_struct import RobotInputData,RobotOutputData


LOG_INTERVAL = 3

def commander_loop(commander: Commander, stop_event: Event, sync_sema: Semaphore, ready_sema:Semaphore, interval_s: float = 0.01):
    """
    Control loop that coordinates with ArmComm via sync_event.
    Every time ArmComm finishes sending a command, it triggers sync_event,
    and this loop will respond by calling commander.giveCommand().
    
    Args:
        commander (Commander): the central dispatcher holding all reactor plugins.
        sync_event (Event): ArmComm sets this event after every serial TX/RX round.
        stop_event (Event): global shutdown flag for all processes.
        interval_s (float): optional fallback polling interval if sync_event is not used.
    """

    print("[CommanderLoop] Started.")
    while not stop_event.is_set():
        sync_sema.acquire()     # 等 ArmComm 给许可（完成上一轮）

        commander.giveCommand()

        ready_sema.release()    # 通知 ArmComm：可以用了 ✅

    print("[CommanderLoop] Stopped.")

def Monitor_system(
    robot_data:         RobotInputData,
    cmd_data:           RobotOutputData,
    robot_mode:         Value,
    stop_event:         threading.Event,
) -> None:
    while not stop_event.is_set():
        print("========== [Monitor] Commander Status ==========")

        # 打印 Commander 内部的数据
        print("-> Commander.robot_data:")
        toprint = {
            "Main Robot data": robot_data.to_dict(),
            "Main Command data": cmd_data.to_dict(),
        }
        nice_print_sections(toprint)

        time.sleep(LOG_INTERVAL)

    logging.info("[Monitor_system] Monitoring stopped.")


