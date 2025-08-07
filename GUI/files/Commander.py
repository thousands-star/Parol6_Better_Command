from typing import Dict, List, Callable
from Reactor import Reactor
from tools.shared_struct import RobotInputData, RobotOutputData
from action import Action  # 你刚才改名后的基类
from action import DummyAction
import threading
from multiprocessing import Value
from enum import IntEnum, auto


# 1) 定义模式
class Mode(IntEnum):
    GUI        = 0
    FOLLOW_TAG = 1
    AUTO_NAV   = 2
    API        = 3

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
        self.max_queue_length = 1000
        self._enabled = Value("i", 1)  # 1: enabled, 0: disabled

    def giveCommand(self):
        # 1. 执行队头 Action 的一步
    # 1. 如果启用了控制并且队列非空，执行队头动作
        if self._enabled.value and self.action_queue:
            self.current_action = self.action_queue[0]
            stateless_fn = self.current_action.step(self.robot_data, self.cmd_data)
            if stateless_fn:
                stateless_fn(self.robot_data, self.cmd_data)
            if self.current_action.is_done():
                self.action_queue.pop(0)

        # 2. 自动补充新动作（如果不在 API 模式下）
        if (self.get_mode() is not Mode.API) and len(self.action_queue) < self.max_queue_length:
            reactor = self.plugins.get(self.current_mode.value)
            if reactor:
                new_action = reactor.giveCommand(self.robot_data)
                if new_action:
                    self.action_queue.append(new_action)

        if (Mode(self.get_mode()) is Mode.API) and len(self.action_queue) == 0:
            self.action_queue.append(DummyAction(self.robot_data.position))

        #return self.cmd_data.pack()


    def inject_action(self, action: Action) -> bool:
        """API 注入动作"""
        print(type(action).__name__)
        if len(self.action_queue) < self.max_queue_length:
            self.action_queue.append(action)
            return True
        return False
    
    def get_action_queue(self) -> List[Dict]:
        toReturn = []
        for action in self.action_queue:
            toReturn.append({"type":type(action).__name__})
        return toReturn
       

    def set_enable(self, flag: bool):
        self._enabled.value = int(flag)
    
    def get_mode(self):
        return self.current_mode.value

    def set_mode(self, mode):
        self.current_mode.value = mode

    def is_enabled(self) -> bool:
        return bool(self._enabled.value)

    def clear_queue(self):
        """清空当前动作队列（如遇紧急情况）"""
        self.action_queue.clear()

    def should_auto_fill(self) -> bool:
        """是否允许 reactor 自动补充指令"""
        return self.current_mode.value != Mode.API

    def to_dict(self) -> Dict[str, any]:
        """返回当前状态字典"""
        if self.current_mode.value == 3:
            mode = 0
        else:
            mode = self.current_mode.value
        return {
            "robot_data": self.robot_data.to_dict(),
            "cmd_data": self.cmd_data.to_dict(),
            Mode(mode).name: self.plugins[mode].to_dict(),
            "commander internal": {
                "queue length": len(self.action_queue),
                "enabled": self._enabled.value,
                "mode": self.current_mode.value,
                "current action": self.current_action,
            }
        }