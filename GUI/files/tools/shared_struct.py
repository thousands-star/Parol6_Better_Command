from dataclasses import dataclass, field
from multiprocessing import Array, Value
from typing import List, Dict, Any

# Robot Input Data is the input we are getting from robot.
# It was utilized in the receiving thread in serial_sender_latest.
@dataclass
class RobotInputData:
    position:          Array = field(default_factory=lambda: Array("i", 6, lock=False))
    speed:             Array = field(default_factory=lambda: Array("i", 6, lock=False))
    homed:             Array = field(default_factory=lambda: Array("i", 8, lock=False))
    inout:             Array = field(default_factory=lambda: Array("i", 8, lock=False))
    temperature_error: Array = field(default_factory=lambda: Array("i", 8, lock=False))
    position_error:    Array = field(default_factory=lambda: Array("i", 8, lock=False))
    timeout_error:     Value = field(default_factory=lambda: Value("i", 0))
    timing_data:       Value = field(default_factory=lambda: Value("i", 0))
    xtr_data:          Value = field(default_factory=lambda: Value("i", 0))
    gripper_data:      Array = field(default_factory=lambda: Array("i", 6, lock=False))

    def initialize(
        self,
        position_init:          List[int] = None,
        speed_init:             List[int] = None,
        homed_init:             List[int] = None,
        inout_init:             List[int] = None,
        temperature_error_init: List[int] = None,
        position_error_init:    List[int] = None,
        gripper_init:           List[int] = None,
    ):
        """Fill each Array from the given list (or zeros if None)."""
        def fill(arr: Array, data: List[int]):
            default = [0] * len(arr)
            src = data if data is not None else default
            for i, v in enumerate(src):
                arr[i] = v

        fill(self.position,          position_init)
        fill(self.speed,             speed_init)
        fill(self.homed,             homed_init)
        fill(self.inout,             inout_init)
        fill(self.temperature_error, temperature_error_init)
        fill(self.position_error,    position_error_init)
        fill(self.gripper_data,      gripper_init)

    def to_dict(self) -> Dict[str, Any]:
        """Snapshot of all fields as plain Python types."""
        return {
            "position":          list(self.position),
            "speed":             list(self.speed),
            "homed":             list(self.homed),
            "inout":             list(self.inout),
            "temperature_error": list(self.temperature_error),
            "position_error":    list(self.position_error),
            "timeout_error":     self.timeout_error.value,
            "timing_data":       self.timing_data.value,
            "xtr_data":          self.xtr_data.value,
            "gripper_data":      list(self.gripper_data),
        }

@dataclass
class RobotOutputData:
    position:         Array = field(default_factory=lambda: Array("i", 6, lock=False))
    speed:            Array = field(default_factory=lambda: Array("i", 6, lock=True))
    command:          Value = field(default_factory=lambda: Value("i", 0))
    affected_joint:   Array = field(default_factory=lambda: Array("i", 8, lock=False))
    inout:            Array = field(default_factory=lambda: Array("i", 8, lock=False))
    timeout:          Value = field(default_factory=lambda: Value("i", 0))
    gripper_data:     Array = field(default_factory=lambda: Array("i", 6, lock=False))

    def initialize(
        self,
        position_init:       List[int] = None,
        speed_init:          List[int] = None,
        affected_joint_init: List[int] = None,
        inout_init:          List[int] = None,
        gripper_init:        List[int] = None,
        command_init:        int = 0,
        timeout_init:        int = 0,
    ):
        """Fill each Array/Value from the given list/int (or zeros if None)."""
        def fill(arr: Array, data: List[int]):
            default = [0] * len(arr)
            src = data if data is not None else default
            for i, v in enumerate(src):
                arr[i] = v

        fill(self.position,       position_init)
        fill(self.speed,          speed_init)
        fill(self.affected_joint, affected_joint_init)
        fill(self.inout,          inout_init)
        fill(self.gripper_data,   gripper_init)
        self.command.value = command_init
        self.timeout.value = timeout_init

    def to_dict(self) -> Dict[str, Any]:
        """Snapshot of all fields as plain Python types."""
        return {
            "position":       list(self.position),
            "speed":          list(self.speed),
            "command":        self.command.value,
            "affected_joint": list(self.affected_joint),
            "inout":          list(self.inout),
            "timeout":        self.timeout.value,
            "gripper_data":   list(self.gripper_data),
        }

def main():
    print("===== RobotInputData TEST =====")
    input_data = RobotInputData()
    input_data.initialize(
        position_init=[31, 32, 33, 34, 35, 36],
        speed_init=[41, 42, 43, 44, 45, 46],
        homed_init=[1] * 8,
        inout_init=[1] * 8,
        temperature_error_init=[0, 1, 0, 1, 0, 1, 0, 1],
        position_error_init=[0, 0, 1, 1, 0, 0, 1, 1],
        gripper_init=[10, 20, 30, 40, 50, 60]
    )

    print("→ Initial snapshot:")
    print(input_data.to_dict())

    # Modify in-place
    input_data.position[2] = 123
    input_data.speed[5] = 99
    input_data.timeout_error.value = 7
    input_data.gripper_data[0] = 77

    print("\n→ After modification:")
    print(input_data.to_dict())

    print("\n===== RobotOutputData TEST =====")
    output_data = RobotOutputData()
    output_data.initialize(
        position_init=[1, 11, 111, 1111, 11111, 10],
        speed_init=[2, 21, 22, 23, 24, 25],
        command_init=42,
        affected_joint_init=[1, 0, 1, 0, 1, 0, 1, 0],
        inout_init=[0, 1, 0, 1, 0, 1, 0, 1],
        timeout_init=88,
        gripper_init=[1, 1, 1, 1, 2, 3]
    )

    print("→ Initial snapshot:")
    print(output_data.to_dict())

    # Modify in-place
    output_data.command.value = 99
    output_data.speed[0] = 777
    output_data.gripper_data[5] = 66

    print("\n→ After modification:")
    print(output_data.to_dict())


if __name__ == "__main__":
    main()
