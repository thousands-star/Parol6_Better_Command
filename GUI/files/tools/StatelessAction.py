# tools/stateless_action.py

from typing import List
from .shared_struct import RobotOutputData, RobotInputData
from . import PAROL6_ROBOT

def move_joints(cmd_data: RobotOutputData, robot_data: RobotInputData,
                joint_ids: List[int], speeds: List[int]) -> str:
    """
    Stateless joint speed control for multiple joints.
    Returns a string log describing the outcome.
    """
    assert len(joint_ids) == len(speeds), "joint_ids and speeds must match"

    cmd_data.speed = [0] * 6
    log_msgs = []

    for jid, spd in zip(joint_ids, speeds):
        pos = robot_data.position[jid]
        min_lim, max_lim = PAROL6_ROBOT.Joint_limits_steps[jid]

        if (spd > 0 and pos >= max_lim) or (spd < 0 and pos <= min_lim):
            log_msgs.append(f"[SKIP:{jid}] out-of-range (pos={pos})")
            continue

        cmd_data.speed[jid] = spd
        log_msgs.append(f"Joint {jid + 1} → {spd}")

    cmd_data.command.value = 123

    if log_msgs:
        return "MoveJoints: " + "; ".join(log_msgs)
    else:
        return "MoveJoints: No valid joint"

def stop_all_joints(cmd_data: RobotOutputData):
    cmd_data.speed = [0] * 6
    cmd_data.command.value = 123  # Standard command code for speed move