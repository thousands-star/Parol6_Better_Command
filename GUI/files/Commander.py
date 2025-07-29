from typing import Dict, List, Callable
from Reactor import Reactor
from tools.shared_struct import RobotInputData, RobotOutputData
from action import Action  # 你刚才改名后的基类
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
        self.cmd_data     = cmd_data
        self.robot_data   = robot_data
        self.plugins      = plugins
        self.current_mode = current_mode

        self.action_queue: List[Action] = []
        self.current_action: Action | None = None
        self.max_queue_length = 10

    def giveCommand(self):
        # 1. 执行队头 Action 的一步
        if self.action_queue:
            current_action = self.action_queue[0]
            stateless_fn = current_action.step(self.robot_data, self.cmd_data)
            if stateless_fn:
                stateless_fn(self.robot_data, self.cmd_data)
            if current_action.is_done():
                self.action_queue.pop(0)

        # 2. If queue is not full, get something from current reactor.
        if len(self.action_queue) < self.max_queue_length:
            reactor = self.plugins.get(self.current_mode.value)
            new_action = reactor.giveCommand(self.robot_data)
            if new_action:
                self.action_queue.append(new_action)

        return self.cmd_data.pack()


    def to_dict(self) -> Dict[str, any]:
        return {
            "robot_data": self.robot_data.to_dict(),
            "cmd_data": self.cmd_data.to_dict(),
            Mode(self.current_mode.value).name: self.plugins[self.current_mode.value].to_dict()
        }