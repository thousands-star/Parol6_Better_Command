# tools/action.py
from abc import ABC, abstractmethod
import time
from tools.shared_struct import RobotInputData, RobotOutputData
import tools.PAROL6_ROBOT as PAROL6_ROBOT
from tools.speed_tools import percent_to_joint_speed
from tools.StatelessAction import move_joints, cartesian_jog, dummy_data
import roboticstoolbox as rp
from roboticstoolbox.tools.trajectory import trapezoidal

class Action(ABC):
    modal:bool = False
    @abstractmethod
    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        """Returns a callable like: lambda rdata, cdata: ..."""
        pass

    @abstractmethod
    def is_done(self) -> bool:
        """Should return True when the action is completed"""
        pass
    def name(self) -> str:
        """返回动作类名（可被子类覆盖以返回更友好的名称）"""
        return self.__class__.__name__
    

class SingleJointJogAction(Action):
    modal = False
    def __init__(self, joint_id: int, speed: int, shared_string = None):
        self.joint_id = joint_id
        self.speed = speed
        self.shared_string = shared_string
        self.done = False

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done:
            return None
        
        self.done = True
        msg = move_joints(cmd_data, robot_data, [self.joint_id], [self.speed])
        
        if self.shared_string is not None:
            self.shared_string.value = msg.encode()[:120]

        return None  # 已经完成动作了，不需要额外的 stateless_fn
    
    def is_done(self):
        return self.done
    
class TimedJointJogAction(Action):
    modal = True

    def __init__(self, joint_id: int, percent: float, direction: int, duration_s: float, shared_string=None):
        """
        Args:
            joint_id (int): 关节索引 (0–5)
            percent (float): 速度百分比 (0–100)
            direction (int): 方向 (+1 或 -1)
            duration_s (float): 运行时长
            shared_string (multiprocessing.Value, optional): 状态信息
        """
        super().__init__()
        self.joint_id = joint_id
        self.speed = percent_to_joint_speed(joint_id, percent, direction)
        self.deadline = time.time() + duration_s
        self.done = False
        self.shared_string = shared_string

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        # 1. 限位检测
        if ((robot_data.position[self.joint_id] <= PAROL6_ROBOT.Joint_limits_steps[self.joint_id][0] 
             and self.speed < 0) or
            (robot_data.position[self.joint_id] >= PAROL6_ROBOT.Joint_limits_steps[self.joint_id][1] 
             and self.speed > 0)):
            self.done = True
            if self.shared_string:
                self.shared_string.value = b'Limit reached, stopped'
            return None

        # 2. 时间截止
        if time.time() >= self.deadline:
            self.done = True
            if self.shared_string:
                self.shared_string.value = b'TimedJog finished'
            return None

        # 3. 正常发速度
        def fn(rdata, cdata):
            for i in range(6):
                cdata.speed[i] = 0
            cdata.speed[self.joint_id] = self.speed
            cdata.command.value = 123
            if self.shared_string:
                try:
                    self.shared_string.value = b'TimedJog active'
                except Exception:
                    pass
        return fn
    
    def is_done(self):
        return self.done
        
    
class SingleCartesianJogAction(Action):
    modal = False
    def __init__(
        self,
        dx: float = 0, dy: float = 0, dz: float = 0,
        rx: float = 0, ry: float = 0, rz: float = 0,
        frame: str = "WRF",
        speed_pct: float = 50,
        interval_s: float = 0.01,
        shared_string=None
    ):
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.frame = frame
        self.speed_pct = speed_pct
        self.interval_s = interval_s
        self.shared_string = shared_string
        self.done = False

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done:
            return None

        self.done = True
        msg = cartesian_jog(
            cmd_data=cmd_data,
            robot_data=robot_data,
            dx=self.dx,
            dy=self.dy,
            dz=self.dz,
            rx=self.rx,
            ry=self.ry,
            rz=self.rz,
            frame=self.frame,
            speed_pct=self.speed_pct,
            interval_s=self.interval_s
        )
        if self.shared_string is not None:
            self.shared_string.value = msg.encode()[:120] 
        return None

    def is_done(self):
        return self.done

# 将下面的类添加到 action.py（靠近其他 Action 类处）
import time  # 如果 action.py 还没有 import time，请添加

class TimedCartesianJogAction(Action):
    """
    在 duration_s 秒内重复发送 cartesian_jog，每次由 Commander 调用 stateless_fn。
    - dx,dy,dz：每次调用的位移（m）
    - rx,ry,rz：每次调用的角度（deg）
    - frame：'WRF' 或 'TRF'
    - speed_pct：百分比速度（0..100）
    - interval_s：用于 IK 计算的时间步长（应与 control loop interval 一致）
    - duration_s：总执行时长（秒）
    - shared_string：可选，写入日志/状态的 multiprocessing.Array 引用
    """
    modal = True

    def __init__(self,
                 dx: float = 0, dy: float = 0, dz: float = 0,
                 rx: float = 0, ry: float = 0, rz: float = 0,
                 frame: str = "WRF",
                 speed_pct: float = 50,
                 interval_s: float = 0.01,
                 duration_s: float = 1.0,
                 shared_string = None):
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.frame = frame
        self.speed_pct = float(speed_pct)
        self.interval_s = float(interval_s)
        self.duration_s = float(duration_s)
        self.shared_string = shared_string

        self.started = False
        self.end_time = None
        self.done = False

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        # 如果已经完成，什么都不做
        if self.done:
            return None

        now = time.time()
        if not self.started:
            self.end_time = now + self.duration_s
            self.started = True

        # 返回一个 stateless 函数，由 Commander 每个周期调用
        def _stateless(rdata: RobotInputData, cdata: RobotOutputData):
            # 每周期调用 cartesian_jog（函数内会做 IK -> joint speeds -> 写入 cdata.speed）
            msg = cartesian_jog(
                cmd_data=cdata,
                robot_data=rdata,
                dx=self.dx,
                dy=self.dy,
                dz=self.dz,
                rx=self.rx,
                ry=self.ry,
                rz=self.rz,
                frame=self.frame,
                speed_pct=self.speed_pct,
                interval_s=self.interval_s
            )
            # 可选写日志显示（长度裁切以适配 shared_string）
            if self.shared_string is not None:
                try:
                    self.shared_string.value = msg.encode()[:120]
                except Exception:
                    # 忽略写 shared_string 的错误（防止阻塞）
                    pass

            # 检查是否到时间
            if time.time() >= self.end_time:
                self.done = True

        return _stateless

    def is_done(self):
        return self.done


    
class HomeRobotAction(Action):
    modal = True
    def __init__(self, shared_string):
        self.done = False
        self.started = False
        self.shared_string = shared_string

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        # Step 1: If already done, do nothing
        if self.done:
            return

        # Step 2: If not started yet, send homing command once
        if not self.started:
            cmd_data.command.value = 100  # HOME
            self.shared_string.value = b"Log: Robot homing"
            self.started = True
            return
        elif all(h == 1 for h in robot_data.homed[:6]):
                self.shared_string.value = b"Log: Robot homing done"
                self.done = True
        else:
        # Step 4: if not homed but started, toggle command.value to 255.
            cmd_data.command.value = 255 # Wait.

    def is_done(self):
        return self.done
    
class EnableRobotAction(Action):
    modal = False
    def __init__(self, shared_string):
        self.done = False
        self.shared_string = shared_string

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done: return
        cmd_data.command.value = 101
        self.shared_string.value = b"Log: Robot enable"
        self.done = True

    def is_done(self): return self.done


class DisableRobotAction(Action):
    modal = False
    def __init__(self, shared_string):
        self.done = False
        self.shared_string = shared_string

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done: return
        cmd_data.command.value = 102
        self.shared_string.value = b"Log: Robot disable"
        self.done = True

    def is_done(self): return self.done


class ClearErrorAction(Action):
    modal = False
    def __init__(self, shared_string):
        self.done = False
        self.shared_string = shared_string

    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done: return
        cmd_data.command.value = 103
        self.shared_string.value = b"Log: Error clear"
        self.done = True

    def is_done(self): return self.done

class DummyAction(Action):
    modal = False
    def __init__(self, position):
        self.done = False
        self.position = position

    def step(self, robot_data:RobotInputData, cmd_data:RobotOutputData):
        if self.done: return
        dummy_data(cmd_data.position, cmd_data.speed, cmd_data.command, self.position)
        self.done = True

    def is_done(self):
        return self.done
    
import numpy as np
import roboticstoolbox as rp
from roboticstoolbox.tools.trajectory import trapezoidal
import tools.PAROL6_ROBOT as PAROL6_ROBOT

class Move2JointsAction(Action):
    """
    Move to joint targets by streaming small trajectory slices.

    Args:
      q_deg: 6 元素关节角(°)
      t:     总时长(秒). t>0 时优先生效
      v_pct: 速度百分比[0..100]（当 t 不给或为 0 时使用）
      a_pct: 加速度百分比[0..100]（同上）
      profile: "poly" | "trap"  （t>0 时选择 jtraj 或 trapezoidal）
      tracking: "pos" | "speed" （156=pos, 123=speed）
      interval_s: 控制周期，默认 0.01s
      tolerance_steps: 结束判定步数阈值
      shared_string: 可选日志输出
    """
    modal = True

    def __init__(self,
                 q_deg,
                 t: float | None = None,
                 v_pct: float | None = None,
                 a_pct: float | None = None,
                 profile: str = "poly",
                 tracking: str = "pos",
                 interval_s: float = 0.01,
                 tolerance_steps: int = 6,
                 shared_string=None):
        
        self.q_deg = [float(x) for x in q_deg]; assert len(self.q_deg) == 6
        self.t = float(t) if (t is not None) else 0.0
        self.v_pct = None if v_pct is None else float(np.clip(v_pct, 0, 100))
        self.a_pct = None if a_pct is None else float(np.clip(a_pct, 0, 100))
        self.profile = (profile or "poly").lower()
        self.tracking = (tracking or "pos").lower()
        self.interval_s = max(float(interval_s), 1e-3)
        self.tolerance_steps = int(max(1, tolerance_steps))
        self.shared_string = shared_string

        self.started = False
        self.done = False
        self.k = 0
        self.N = 0
        self.pos_seq = None   # (N,6)  in steps
        self.spd_seq = None   # (N,6)  in steps/s

    # ---- helpers ----
    @staticmethod
    def _within_limits_rad(q_rad):
        for i in range(6):
            lo, hi = PAROL6_ROBOT.Joint_limits_radian[i]
            if not (lo <= q_rad[i] <= hi):
                return False, i
        return True, -1

    def _plan_with_duration(self, q0_rad, q1_rad):
        """
        t > 0: 用 jtraj 或 trapezoidal 生成 Q/Qd（单位: rad 和 rad/s），然后转 steps 与 step/s
        """
        N = max(2, int(np.ceil(self.t / self.interval_s)))
        tt = np.linspace(0.0, float(self.t), N)  # 统一用真实时间轴，qd 就是对时间的导数

        if self.profile == "trap":
            # mtraj 返回 Trajectory 对象
            traj = rp.tools.trajectory.mtraj(rp.tools.trajectory.trapezoidal, q0_rad, q1_rad, tt)
            Q  = np.asarray(traj.q)   # (N,6) in rad
            Qd = np.asarray(traj.qd)  # (N,6) in rad/s
        else:
            # jtraj 也返回 Trajectory 对象
            traj = rp.tools.trajectory.jtraj(q0_rad, q1_rad, tt)
            Q  = np.asarray(traj.q)   # (N,6) in rad
            Qd = np.asarray(traj.qd)  # (N,6) in rad/s

            # 保险：有些版本可能不给 qd（极少见），就用数值微分补一下
            if Qd is None or np.size(Qd) == 0:
                dt = np.gradient(tt)
                Qd = np.vstack([np.zeros((1, Q.shape[1])), np.diff(Q, axis=0)]) / dt.reshape(-1, 1)

        # 转 step 与 step/s
        pos_steps = np.zeros_like(Q)
        spd_steps = np.zeros_like(Qd)
        for i in range(6):
            pos_steps[:, i] = [PAROL6_ROBOT.RAD2STEPS(float(q), i) for q in Q[:, i]]
            spd_steps[:, i] = [PAROL6_ROBOT.SPEED_RAD2STEP(float(w), i) for w in Qd[:, i]]

        return pos_steps.astype(int), spd_steps.astype(int)

    def _plan_with_va(self, q0_steps, q1_steps):
        """v%/a%：在“步”域构造每轴同步的 trapezoid"""
        dsteps = np.abs(q1_steps - q0_steps)
        lead = int(np.argmax(dsteps))
        if dsteps[lead] < 1:
            return np.array([[int(x) for x in q0_steps]]), np.zeros((1,6), dtype=int)

        v_lead = np.interp(self.v_pct if self.v_pct is not None else 45.0,
                           [0, 100],
                           [PAROL6_ROBOT.Joint_min_speed[lead], PAROL6_ROBOT.Joint_max_speed[lead]])
        a_abs  = np.interp(self.a_pct if self.a_pct is not None else 27.0,
                           [0, 100],
                           [PAROL6_ROBOT.Joint_min_acc, PAROL6_ROBOT.Joint_max_acc])

        tacc = float(v_lead) / float(a_abs)
        total_t = float(dsteps[lead]) / float(max(1.0, v_lead)) + tacc
        tt = np.arange(0.0, total_t, self.interval_s)
        N = len(tt)

        pos_steps = np.tile(q0_steps, (N, 1)).astype(float)
        spd_steps = np.zeros((N, 6), dtype=float)

        for i in range(6):
            if dsteps[i] < 1:  # 不动
                continue
            vi = float(dsteps[i]) / max(1e-6, (total_t - tacc))  # 确保同一时刻到达
            try:
                traj_i = trapezoidal(float(q0_steps[i]), float(q1_steps[i]), tt, V=vi)
                qi = np.asarray(traj_i.q).reshape(-1)
                qdi = np.asarray(traj_i.qd).reshape(-1)
            except Exception:
                traj_i = trapezoidal(float(q0_steps[i]), float(q1_steps[i]), tt)
                qi = np.asarray(traj_i.q).reshape(-1)
                qdi = np.asarray(traj_i.qd).reshape(-1)
            pos_steps[:, i] = qi
            spd_steps[:, i] = qdi

        return pos_steps.astype(int), spd_steps.astype(int)

    # ---- runtime ----
    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        if self.done:
            return
    

        if not self.started:
            q0_rad = np.array([PAROL6_ROBOT.STEPS2RADS(robot_data.position[i], i) for i in range(6)], dtype=float)
            q1_rad = np.array([PAROL6_ROBOT.DEG2RAD(self.q_deg[i]) for i in range(6)], dtype=float)

            ok, bad = self._within_limits_rad(q1_rad)
            if not ok:
                if self.shared_string is not None:
                    self.shared_string.value = f"Error: target joint {bad} out of range".encode()[:120]
                print("If I were here.")
                cmd_data.command.value = 255
                self.done = True
                return

            if self.t and self.t > 0:
                pos_steps, spd_steps = self._plan_with_duration(q0_rad, q1_rad)
            else:
                q0_steps = np.array([robot_data.position[i] for i in range(6)], dtype=float)
                q1_steps = np.array([PAROL6_ROBOT.RAD2STEPS(q1_rad[i], i) for i in range(6)], dtype=float)
                pos_steps, spd_steps = self._plan_with_va(q0_steps, q1_steps)

            spd_steps = np.vstack([np.zeros((1, 6), dtype=int),
                                np.diff(pos_steps, axis=0)]).astype(int)
            self.N = int(pos_steps.shape[0])
            self.pos_seq = pos_steps
            self.spd_seq = spd_steps
            self.k = 0
            for i in range(6):
                cmd_data.affected_joint[i] = 1

            pos_steps = pos_steps.astype(int)


            if self.shared_string is not None:
                self.shared_string.value = f"Log: Move2Joints N={self.N}, tracking={self.tracking}".encode()[:120]
            self.started = True

        # 推送第 k 帧
        if self.k < self.N:
            for i in range(6):
                cmd_data.position[i] = int(self.pos_seq[self.k, i])
                cmd_data.speed[i]    = int(self.spd_seq[self.k, i])  # <- 每tick步差
            cmd_data.timeout.value = 1                               # <- 维持激活（1~2 都行）
            cmd_data.command.value = 123 if self.tracking == "speed" else 156
            self.k += 1
            return

        # 末尾：保持最终点，等“收敛”再结束
        for i in range(6):
            cmd_data.position[i] = int(self.pos_seq[-1, i])
            cmd_data.speed[i] = 0
        cmd_data.command.value = 255  # idle

        stable = True
        for i in range(6):
            if abs(int(robot_data.position[i]) - int(self.pos_seq[-1, i])) > self.tolerance_steps:
                stable = False
                break
        if stable:
            self.done = True
            if self.shared_string is not None:
                self.shared_string.value = b"Log: Move2Joints done"

    def is_done(self) -> bool:
        return self.done


