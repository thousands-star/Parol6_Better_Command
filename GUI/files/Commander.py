from typing import Dict
from Reactor import Reactor
from tools.shared_struct import RobotInputData, RobotOutputData
import threading
from multiprocessing import Value
from enum import IntEnum, auto


# 1) 定义模式
class Mode(IntEnum):
    GUI        = 0
    FOLLOW_TAG = 1
    AUTO_NAV   = 2

class Commander:
    def __init__(self,
                 cmd_data:     RobotOutputData,
                 robot_data:   RobotInputData,
                 plugins:      Dict[Mode, Reactor],
                 current_mode: Value,
                 ):
        
        self.cmd_data    = cmd_data
        self.robot_data  = robot_data
        self.plugins     = plugins
        self.current_mode= current_mode

    def giveCommand(self):
        """在共享的 self.cmd_data 上就地写入下一个命令。"""
        reactor = self.plugins.get(self.current_mode.value)
        return reactor.giveCommand(robot_data= self.robot_data, cmd_data= self.cmd_data)

    def to_dict(self) -> Dict[str, any]:
        """
        Return a dictionary contains current state.
        """
        base = {
            "robot_data": self.robot_data.to_dict(),
            "cmd_data": self.cmd_data.to_dict(),
            Mode(self.current_mode.value).name: self.plugins[self.current_mode.value].to_dict()
        }
       
        return base  