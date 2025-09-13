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
    speed_pct = float(np.clip(speed_pct, 0.0, 100.0))
    interval_s = max(float(interval_s), 1e-3)

    # 当前关节角 -> 齐次位姿
    q1 = np.array([PAROL6_ROBOT.STEPS2RADS(robot_data.position[i], i) for i in range(6)])
    T  = PAROL6_ROBOT.robot.fkine(q1)

    # 位置增量
    d = np.array([dx, dy, dz], dtype=float)
    if frame.upper() == "WRF":
        T.t = T.t + d
        log = f"Log: Cartesian WRF jog Δpos=({dx:.4f}, {dy:.4f}, {dz:.4f})"
    elif frame.upper() == "TRF":
        T.t = T.t + T.R @ d
        log = f"Log: Cartesian TRF jog Δpos=({dx:.4f}, {dy:.4f}, {dz:.4f})"
    else:
        return "Error: Unknown reference frame"

    # 姿态增量
    if rx != 0:
        T = T * SE3.Rx(rx, unit='deg'); log += f", Rx={rx:.1f}°"
    if ry != 0:
        T = T * SE3.Ry(ry, unit='deg'); log += f", Ry={ry:.1f}°"
    if rz != 0:
        T = T * SE3.Rz(rz, unit='deg'); log += f", Rz={rz:.1f}°"

    # IK
    sol = PAROL6_ROBOT.robot.ikine_LMS(T, q0=q1, ilimit=10)
    if not getattr(sol, "success", False):
        for i in range(6): 
            cmd_data.speed[i] = 0
        cmd_data.command.value = 123
        return "Warning: IK failed"

    q2    = sol.q if hasattr(sol, 'q') else sol[0]
    q2[5] = q1[5]
    dq_dt = (q2 - q1) / interval_s  # rad/s
    step_speeds = np.array([PAROL6_ROBOT.SPEED_RAD2STEP(dq_dt[i], i) for i in range(6)], dtype=float)

    # 无需动作
    if float(np.max(np.abs(step_speeds))) == 0.0:
        for i in range(6):
            cmd_data.speed[i] = 0
        cmd_data.command.value = 123
        return "Log: No motion required"

    # per-joint limits
    vmin_vec = np.asarray(PAROL6_ROBOT.Joint_min_jog_speed, dtype=float)
    vmax_vec = np.asarray(PAROL6_ROBOT.Joint_max_jog_speed, dtype=float)

    alpha     = speed_pct / 100.0
    target_lim= vmin_vec + alpha * (vmax_vec - vmin_vec)  # 每轴上限

    # —— 全局缩放，保持方向不变 ——
    # 允许的缩放= min_i (target_lim[i] / |v[i]|)（仅对非零项取）
    abs_v   = np.abs(step_speeds)
    with np.errstate(divide='ignore', invalid='ignore'):
        ratios = np.where(abs_v > 1e-9, target_lim / abs_v, np.inf)
    scale = float(np.clip(np.min(ratios), 0.0, 1.0))  # ≤1 表示需要缩小
    step_speeds = step_speeds * scale

    # —— 静区滤波（温和一些） ——
    # 用常数死区（比如 20 steps/s），避免太激
    DEAD_CONST = 20.0
    cmd_data.command.value = 123

    def round_away_from_zero(x: float) -> int:
        return int(np.sign(x) * np.floor(abs(x) + 0.5))

    # 写入各轴（保持方向；若低于死区则置0；若非零但太小，抬到各轴 vmin）
    for i in range(6):
        v = float(step_speeds[i])
        if abs(v) < DEAD_CONST:
            cmd_data.speed[i] = 0
            continue
        sgn = np.sign(v)
        mag = abs(v)
        # 非零则至少各轴 vmin，最多 target_lim（按比例缩过，一般不需要再裁）
        mag = min(max(mag, float(vmin_vec[i])), float(target_lim[i]))
        cmd_data.speed[i] = round_away_from_zero(sgn * mag)

    # 只有当没有角度命令时，才把 J6 清零（如你确实想锁 J6）
    if rx == 0 and ry == 0 and rz == 0:
        # 若你想固定 J6，不随动：保留下面一行；否则删除
        # cmd_data.speed[5] = 0
        pass

    return log
