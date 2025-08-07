# tools/stateless_action.py

from typing import List
from .shared_struct import RobotOutputData, RobotInputData
from . import PAROL6_ROBOT
import numpy as np

def move_joints(cmd_data: RobotOutputData, robot_data: RobotInputData,
                joint_ids: List[int], speeds: List[int]) -> str:
    """
    Stateless joint speed control for multiple joints.
    Returns a string log describing the outcome.
    """
    assert len(joint_ids) == len(speeds), "joint_ids and speeds must match"

    for i in range(6):
        cmd_data.speed[i] = 0

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
    for i in range(6):
        cmd_data.speed[i] = 0
    cmd_data.command.value = 123  # Standard command code for speed move

def dummy_data(Position_out,Speed_out,Command_out,Position_in):
    Command_out.value = 255
    for i in range(6):
        Position_out[i] = Position_in[i]
        Speed_out[i] = 0

def cartesian_jog(cmd_data: RobotOutputData,
                  robot_data: RobotInputData,
                  dx: float, dy: float, dz: float,
                  rx: float = 0, ry: float = 0, rz: float = 0,
                  frame: str = "WRF",
                  speed_pct: float = 50.0,
                  interval_s: float = 0.01) -> str:

    q1 = np.array([PAROL6_ROBOT.STEPS2RADS(robot_data.position[i], i) for i in range(6)])
    T = PAROL6_ROBOT.robot.fkine(q1)

    if frame.upper() == "WRF":
        T.t += np.array([dx, dy, dz])
        log = f"Log: Cartesian WRF jog Δpos=({dx:.4f}, {dy:.4f}, {dz:.4f})"
    elif frame.upper() == "TRF":
        T.t += T.R @ np.array([dx, dy, dz])
        log = f"Log: Cartesian TRF jog Δpos=({dx:.4f}, {dy:.4f}, {dz:.4f})"
    else:
        return "Error: Unknown reference frame"

    if rx != 0:
        T = T * T.Rx(rx, unit='deg')
        log += f", Rx={rx:.1f}°"
    if ry != 0:
        T = T * T.Ry(ry, unit='deg')
        log += f", Ry={ry:.1f}°"
    if rz != 0:
        T = T * T.Rz(rz, unit='deg')
        log += f", Rz={rz:.1f}°"

    sol = PAROL6_ROBOT.robot.ikine_LMS(T, q0=q1, ilimit=6)
    if not sol.success:
        return "Warning: IK failed"

    q2 = sol.q if hasattr(sol, 'q') else sol[0]
    dq_dt = (q2 - q1) / interval_s  # rad/s

    # Convert to step/s
    step_speeds = np.array([PAROL6_ROBOT.SPEED_RAD2STEP(dq_dt[i], i) for i in range(6)])

    # Scale factor = limit / max(abs(steps))
    abs_max = np.max(np.abs(step_speeds))
    if abs_max == 0:
        cmd_data.speed = [0] * 6
        return "Log: No motion required"

    limit = np.interp(speed_pct, [0, 100],
                      [max(PAROL6_ROBOT.Joint_min_jog_speed), 
                       min(PAROL6_ROBOT.Joint_max_jog_speed)])

    scale = limit / abs_max
    capped_speeds = step_speeds * scale

    cmd_data.command.value = 123
    for i, s in enumerate(capped_speeds):
        cmd_data.speed[i] = int(s)

    return log