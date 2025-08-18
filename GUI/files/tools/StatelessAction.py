# tools/stateless_action.py

from typing import List
from .shared_struct import RobotOutputData, RobotInputData
from . import PAROL6_ROBOT
import numpy as np
from spatialmath import SE3

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

    # clamp
    speed_pct = float(np.clip(speed_pct, 0.0, 100.0))
    interval_s = max(float(interval_s),1e-3)

    # Current radian.
    q1 = np.array([PAROL6_ROBOT.STEPS2RADS(robot_data.position[i], i) for i in range(6)])
    T = PAROL6_ROBOT.robot.fkine(q1)

    d = np.array([dx, dy, dz], dtype=float)
    if frame.upper() == "WRF":
        T.t = T.t = T.t + d
        log = f"Log: Cartesian WRF jog Δpos=({dx:.4f}, {dy:.4f}, {dz:.4f})"
    elif frame.upper() == "TRF":
        T.t = T.t + T.R @ d
        log = f"Log: Cartesian TRF jog Δpos=({dx:.4f}, {dy:.4f}, {dz:.4f})"
    else:
        return "Error: Unknown reference frame"

    if rx != 0:
        T = T * SE3.Rx(rx, unit='deg')
        log += f", Rx={rx:.1f}°"
    if ry != 0:
        T = T * SE3.Ry(ry, unit='deg')
        log += f", Ry={ry:.1f}°"
    if rz != 0:
        T = T * SE3.Rz(rz, unit='deg')
        log += f", Rz={rz:.1f}°"

    sol = PAROL6_ROBOT.robot.ikine_LMS(T, q0=q1, ilimit=10)
    if not getattr(sol, "success", False):
        # 停止并回报
        for i in range(6): 
            cmd_data.speed[i] = 0
        cmd_data.command.value = 123
        
        return "Warning: IK failed"
    
    q2 = sol.q if hasattr(sol, 'q') else sol[0]
    dq_dt = (q2 - q1) / interval_s  # rad/s

    # Convert to step/s
    step_speeds = np.array([PAROL6_ROBOT.SPEED_RAD2STEP(dq_dt[i], i) for i in range(6)], dtype=float)

    # Scale factor = limit / max(abs(steps))
    abs_max = float(np.max(np.abs(step_speeds)))
    if abs_max == 0.0:
        for i in range(6):
            cmd_data.speed[i] = 0
            cmd_data.command.value = 123
        return "Log: No motion required"

    # per-joint limits as vectors
    vmin_vec = np.asarray(PAROL6_ROBOT.Joint_min_jog_speed, dtype=float)  # shape (6,)
    vmax_vec = np.asarray(PAROL6_ROBOT.Joint_max_jog_speed, dtype=float)  # shape (6,)

    # speed_pct -> per-joint target caps (线性插值到每轴区间)
    alpha = float(np.clip(speed_pct, 0.0, 100.0)) / 100.0
    limit_vec = vmin_vec + alpha * (vmax_vec - vmin_vec)  # shape (6,)

    # 可调的“静区”：原始速度太小就当 0，避免抖动/爬行
    DEADBAND_RATIO = 0.3  # 例如 0.3 * vmin，按需要调
    deadband_vec = DEADBAND_RATIO * vmin_vec

    def round_away_from_zero(x: float) -> int:
        return int(np.sign(x) * np.floor(abs(x) + 0.5))

    cmd_data.command.value = 123

    for i in range(6):
        v = float(step_speeds[i])          # IK → step/s（有正负）
        if v == 0.0:
            cmd_data.speed[i] = 0
            continue

        sgn  = np.sign(v)
        mag  = abs(v)

        # 先应用“静区”，太小就别动，防止抖
        if mag < deadband_vec[i]:
            cmd_data.speed[i] = 0
            continue

        # 逐轴裁到自己的上限
        mag = min(mag, float(limit_vec[i]))

        # 非零就至少跑到每轴的最小 jog（避免 int() 砍成 0）
        mag = max(mag, float(vmin_vec[i]))

        cmd_data.speed[i] = round_away_from_zero(sgn * mag)
    cmd_data.speed[5] = 0

    return log