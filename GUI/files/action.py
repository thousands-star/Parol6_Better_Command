# tools/action.py
from abc import ABC, abstractmethod
from tools.shared_struct import RobotInputData, RobotOutputData
from tools.StatelessAction import move_joints, cartesian_jog, dummy_data

class Action(ABC):
    @abstractmethod
    def step(self, robot_data: RobotInputData, cmd_data: RobotOutputData):
        """Returns a callable like: lambda rdata, cdata: ..."""
        pass

    @abstractmethod
    def is_done(self) -> bool:
        """Should return True when the action is completed"""
        pass

class SingleJointJogAction(Action):
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
    
class SingleCartesianJogAction(Action):
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

    
class HomeRobotAction(Action):
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
    def __init__(self, position):
        self.done = False
        self.position = position

    def step(self, robot_data:RobotInputData, cmd_data:RobotOutputData):
        if self.done: return
        dummy_data(cmd_data.position, cmd_data.speed, cmd_data.command, self.position)
        self.done = True

    def is_done(self):
        return self.done

