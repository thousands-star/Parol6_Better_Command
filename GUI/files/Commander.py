from typing import Dict
from Reactor import Reactor
from tools.shared_struct import RobotInputData, RobotOutputData
import threading
from enum import Enum, auto


# 1) 定义模式
class Mode(Enum):
    GUI        = auto()
    FOLLOW_TAG = auto()
    AUTO_NAV   = auto()

class Commander:
    def __init__(self,
                 cmd_data:     RobotOutputData,
                 robot_data:   RobotInputData,
                 plugins:      Dict[Mode, Reactor],
                 default_mode: Mode,
                 ):
        
        self.cmd_data    = cmd_data
        self.robot_data  = robot_data
        self.plugins     = plugins
        self.current_mode= default_mode

    def switch_mode(self, mode: Mode):
        if mode in self.plugins:
            self.current_mode = mode
        else:
            raise ValueError(f"Unknown mode: {mode}")

    def giveCommand(self):
        """在共享的 self.cmd_data 上就地写入下一个命令。"""
        reactor = self.plugins.get(self.current_mode)
        return reactor.giveCommand(robot_data= self.robot_data, cmd_data= self.cmd_data)

    def to_dict(self) -> Dict[str, any]:
        """
        Return a dictionary contains current state.
        """
        base = {
            "robot_data": self.robot_data.to_dict(),
            "cmd_data": self.cmd_data.to_dict(),
            str(self.current_mode): self.plugins[self.current_mode].to_dict()
        }
       
        return base  