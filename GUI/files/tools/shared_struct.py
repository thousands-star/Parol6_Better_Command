from dataclasses import dataclass, field
from multiprocessing import Array, Value
from typing import List, Dict, Any
import struct

# in big endian machines, first byte of binary representation of the multibyte data-type is stored first. 
int_to_3_bytes = struct.Struct('>I').pack # BIG endian order

# data for output string (data that is being sent to the robot)
#######################################################################################
#######################################################################################
start_bytes =  [0xff,0xff,0xff] 
start_bytes = bytes(start_bytes)

end_bytes =  [0x01,0x02] 
end_bytes = bytes(end_bytes)

# Split data to 3 bytes 
def Split_2_3_bytes(var_in):
    y = int_to_3_bytes(var_in & 0xFFFFFF) # converts my int value to bytes array
    return y

# Splits byte to bitfield list
def Split_2_bitfield(var_in):
    #return [var_in >> i & 1 for i in range(7,-1,-1)] 
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]

# Fuses 3 bytes to 1 signed int
def Fuse_3_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<23:
        value -= 1<<24

    return value

# Fuses 2 bytes to 1 signed int
def Fuse_2_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<15:
        value -= 1<<16

    return value

# Fuse bitfield list to byte
def Fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])

# Check if there is element 1 in the list. 
# If yes return its index, if no element is 1 return -1
def check_elements(lst):
    for i, element in enumerate(lst):
        if element == 1:
            return i
    return -1  # Return -1 if no element is 1
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
    
    def unpack(self, data_buffer_list: List[bytes]) -> None:
        """
        Unpack a raw packet payload (list of single‐byte bytes objects)
        directly into this RobotInputData’s shared arrays/values.
        """
        # 1) split the first 36 bytes into joint & speed triplets
        joints = [data_buffer_list[i:i+3] for i in range(0, 18, 3)]
        speeds = [data_buffer_list[i:i+3] for i in range(18, 36, 3)]

        # 2) fuse and store into self.position / self.speed
        for i in range(6):
            p_bytes = b'\x00' + b''.join(joints[i])
            s_bytes = b'\x00' + b''.join(speeds[i])
            self.position[i] = Fuse_3_bytes(p_bytes)
            self.speed[i]    = Fuse_3_bytes(s_bytes)

        # 3) single‐byte flags & errors
        homed_byte   = data_buffer_list[36]
        io_byte      = data_buffer_list[37]
        temp_err_byte= data_buffer_list[38]
        pos_err_byte = data_buffer_list[39]
        # 4) multi‐byte timing
        timing_bytes = data_buffer_list[40:42]
        t_fused = Fuse_3_bytes(b'\x00\x00' + b''.join(timing_bytes))
        self.timing_data.value = t_fused

        # 5) bit‐field unpacking into 8‐element arrays
        self.homed[:]             = Split_2_bitfield(int.from_bytes(homed_byte,   "big"))
        self.inout[:]             = Split_2_bitfield(int.from_bytes(io_byte,      "big"))
        self.temperature_error[:] = Split_2_bitfield(int.from_bytes(temp_err_byte, "big"))
        self.position_error[:]    = Split_2_bitfield(int.from_bytes(pos_err_byte,  "big"))

        # 6) timeout & XTR
        timeout_byte = data_buffer_list[42]
        xtr_byte     = data_buffer_list[43]
        self.timeout_error.value = int.from_bytes(timeout_byte, "big")
        self.xtr_data.value      = int.from_bytes(xtr_byte,     "big")

        # 7) gripper block
        device_id      = data_buffer_list[44]
        grip_pos       = data_buffer_list[45:47]
        grip_spd       = data_buffer_list[47:49]
        grip_cur       = data_buffer_list[49:51]
        status_byte    = data_buffer_list[51]
        obj_det_byte   = data_buffer_list[52]

        self.gripper_data[0] = int.from_bytes(device_id, "big")
        self.gripper_data[1] = Fuse_2_bytes(b'\x00\x00' + b''.join(grip_pos))
        self.gripper_data[2] = Fuse_2_bytes(b'\x00\x00' + b''.join(grip_spd))
        self.gripper_data[3] = Fuse_2_bytes(b'\x00\x00' + b''.join(grip_cur))
        self.gripper_data[4] = int.from_bytes(status_byte,  "big")
        self.gripper_data[5] = int.from_bytes(obj_det_byte, "big")

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
    
    def pack(self) -> List[bytes]:
        """Packs this RobotOutputData into a list of byte segments to send over serial."""
        CRC_BYTE = 228
        LENGTH_BYTE = 52  # excludes the 3 start bytes + this length byte

        out: List[bytes] = []
        out.append(start_bytes)           # start sequence
        out.append(bytes([LENGTH_BYTE]))  # length

        # 1) Position (6 joints × 3 bytes)
        for i in range(6):
            p = Split_2_3_bytes(self.position[i])
            out.append(p[1:4])

        # 2) Speed (6 joints × 3 bytes)
        for i in range(6):
            s = Split_2_3_bytes(self.speed[i])
            out.append(s[1:4])

        # 3) Single-byte command
        out.append(bytes([self.command.value]))

        # 4) Affected-joints bitfield
        out.append(Fuse_bitfield_2_bytearray(self.affected_joint[:]))

        # 5) I/O-outputs bitfield
        out.append(Fuse_bitfield_2_bytearray(self.inout[:]))

        # 6) Timeout (single byte)
        out.append(bytes([self.timeout.value]))

        # 7) Gripper position/speed/current (each 2 bytes)
        g = self.gripper_data
        pos_b = Split_2_3_bytes(g[0]); out.append(pos_b[2:4])
        spd_b = Split_2_3_bytes(g[1]); out.append(spd_b[2:4])
        cur_b = Split_2_3_bytes(g[2]); out.append(cur_b[2:4])

        # 8) Gripper command, mode, ID (1 byte each)
        out.append(bytes([g[3]]))
        out.append(bytes([g[4]]))
        out.append(bytes([g[5]]))

        # 9) CRC + end bytes
        out.append(bytes([CRC_BYTE]))
        out.append(end_bytes)

        return out

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
