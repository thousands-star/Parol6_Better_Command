import logging
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


# Data that we receive from the robot
#Input is data buffer list
#Output is saved to multiproc arrays and variables
## joints(3byte)x6,speed(3byte)x6,homed(byte),I/O(byte),temp_error(byte),position_error(byte),timing_data(2byte),Timeout_error(byte),xtr2(byte)
# Gripper data == Position(2byte),speed(2byte),current(2byte),status(byte),obj_detection(byte),ID(byte)
## CRC(byte),end1(byte),end2(byte)
# Last 2 bytes are end bytes but we dont unpack then since we chech their validity elsewhere
def Unpack_data_test(data_buffer_list):

    Joints = []
    Speed = []

    for i in range(0,18, 3):
        variable = data_buffer_list[i:i+3]
        Joints.append(variable)

    for i in range(18,36, 3):
        variable = data_buffer_list[i:i+3]
        Speed.append(variable)
    
    Homed = data_buffer_list[36]
    IO_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    Timeout_error = data_buffer_list[42]
    xtr2 = data_buffer_list[43]
    device_ID = data_buffer_list[44]
    Gripper_position = data_buffer_list[45:47]
    Gripper_speed = data_buffer_list[47:49]
    Gripper_current = data_buffer_list[49:51]
    Status = data_buffer_list[51]
    object_detection = data_buffer_list[52]
    CRC_byte = data_buffer_list[53]
    endy_byte1 = data_buffer_list[54]
    endy_byte2 = data_buffer_list[55]

    logging.debug("Robot position")
    logging.debug(Joints) 
    logging.debug("Robot speed")
    logging.debug(Speed)
    logging.debug("Robot homed")
    logging.debug(Homed)
    logging.debug("Robot I/O data")
    logging.debug(IO_var)
    logging.debug("Robot temp error data")
    logging.debug(temp_error)
    logging.debug("Robot position error data")
    logging.debug(position_error)
    logging.debug("Robot timig data")
    logging.debug(timing_data)
    logging.debug("Robot timig error data")
    logging.debug(Timeout_error)
    logging.debug("Robot additional byte 2")
    logging.debug(xtr2)
    logging.debug("Gripper device ID")
    logging.debug(device_ID)
    logging.debug("Gripper position")
    logging.debug(Gripper_position)
    logging.debug("Gripper speed")
    logging.debug(Gripper_speed)
    logging.debug("Gripper current")
    logging.debug(Gripper_current)
    logging.debug("Gripper status")
    logging.debug(Status)
    logging.debug("Gripper object detection")
    logging.debug(object_detection)
    logging.debug("CRC byte")
    logging.debug(CRC_byte)
    logging.debug("End byte 1")
    logging.debug(endy_byte1)
    logging.debug("End byte 2")
    logging.debug(endy_byte2)


# Data that we receive from the robot
#Input is data buffer list
#Output is saved to multiproc arrays and variables
## joints(3byte)x6,speed(3byte)x6,homed(byte),I/O(byte),temp_error(byte),position_error(byte),timing_data(2byte),Timeout_error(byte),xtr2(byte)
# Gripper data == Position(2byte),speed(2byte),current(2byte),status(byte),obj_detection(byte),ID(byte)
## CRC(byte),end1(byte),end2(byte)
# Last 2 bytes are end bytes but we dont unpack then since we chech their validity elsewhere
def Unpack_data(data_buffer_list, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in):

    Joints = []
    Speed = []

    for i in range(0,18, 3):
        variable = data_buffer_list[i:i+3] 
        Joints.append(variable)

    for i in range(18,36, 3):
        variable = data_buffer_list[i:i+3]
        Speed.append(variable)


    for i in range(6):
        var =  b'\x00' + b''.join(Joints[i]) 
        Position_in[i] = Fuse_3_bytes(var)
        var =  b'\x00' + b''.join(Speed[i]) 
        Speed_in[i] = Fuse_3_bytes(var)

    Homed = data_buffer_list[36]
    IO_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    Timeout_error_var = data_buffer_list[42]
    xtr2 = data_buffer_list[43]
    device_ID = data_buffer_list[44]
    Gripper_position = data_buffer_list[45:47]
    Gripper_speed = data_buffer_list[47:49]
    Gripper_current = data_buffer_list[49:51]
    Status = data_buffer_list[51]
    object_detection = data_buffer_list[52]
    CRC_byte = data_buffer_list[53]
    endy_byte1 = data_buffer_list[54]
    endy_byte2 = data_buffer_list[55]

    logging.debug("Robot position")
    logging.debug(Joints)
    logging.debug("Robot speed")
    logging.debug(Speed)
    logging.debug("Robot homed")
    logging.debug(Homed)

    temp = Split_2_bitfield(int.from_bytes(Homed,"big"))
    for i in range(8):
        Homed_in[i] = temp[i]

    logging.debug("Robot I/O data")
    logging.debug(IO_var)

    temp = Split_2_bitfield(int.from_bytes(IO_var,"big"))
    for i in range(8):
        InOut_in[i] = temp[i]

    logging.debug("Robot temp error data")
    logging.debug(temp_error)

    temp = Split_2_bitfield(int.from_bytes(temp_error,"big"))
    for i in range(8):
        Temperature_error_in[i] = temp[i]

    logging.debug("Robot position error data")
    logging.debug(position_error)

    temp = Split_2_bitfield(int.from_bytes(position_error,"big"))
    for i in range(8):
        Position_error_in[i] = temp[i]

    logging.debug("Robot timig data")
    logging.debug(timing_data)
    logging.debug("Robot timig data fused")
    var = b'\x00' + b'\x00' + b''.join(timing_data)
    logging.debug(var)
    logging.debug("Robot timig data fused 2")
    var2 = Fuse_3_bytes(var)
    Timing_data_in.value = var2
    logging.debug(var2)
    logging.debug("Timing in ms")
    logging.debug(var2 * 1.4222222e-6)
    logging.debug(var2)
    logging.debug("Robot timig error data")
    logging.debug(Timeout_error_var)

    Timeout_error.value = int.from_bytes(Timeout_error_var,"big")

    logging.debug("Robot additional byte 2")
    logging.debug(xtr2)

    XTR_data.value = int.from_bytes(xtr2,"big")

    logging.debug("Gripper device ID")
    logging.debug(device_ID)

    Gripper_data_in[0] = int.from_bytes(device_ID,"big") 

    logging.debug("Gripper position")
    logging.debug(Gripper_position)

    var =  b'\x00'+ b'\x00' + b''.join(Gripper_position) 
    Gripper_data_in[1] = Fuse_2_bytes(var)

    logging.debug("Gripper speed")
    logging.debug(Gripper_speed)


    var =  b'\x00'+ b'\x00' + b''.join(Gripper_speed) 
    Gripper_data_in[2] = Fuse_2_bytes(var)

    logging.debug("Gripper current")
    logging.debug(Gripper_current)


    var =  b'\x00'+ b'\x00' + b''.join(Gripper_current) 
    Gripper_data_in[3] = Fuse_2_bytes(var)

    logging.debug("Gripper status")
    logging.debug(Status)

    Gripper_data_in[4] = int.from_bytes(Status,"big")

    logging.debug("Gripper object detection")
    logging.debug(object_detection)

    Gripper_data_in[5] = int.from_bytes(object_detection,"big")

    logging.debug("CRC byte")
    logging.debug(CRC_byte)
    logging.debug("End byte 1")
    logging.debug(endy_byte1)
    logging.debug("End byte 2")
    logging.debug(endy_byte2)
  
# Data we send to the robot
# Inputs are multiproc arrays and variables
# Outputs is list of bytes objects? that need to be send by the serial
# Position is needed robot position - it is list of 6 joint position elements and packs to 3 bytes each
# Speed is needed robot position - it is list of 6 joint speed elements and packs to 3 bytes each
# Commad is single byte
# Affected joint is single byte 
# InOut is byte where 
# Timeout is byte  
# Gripper data is list of gripper elements like speed, positon, ID...
# Positon packs to 2 bytes, speed to 2 bytes, current to 2 bytes, command to 1 byte, mode 1 byte, ID to 1  byte

# First 3 bytes are start condition 
# After that is data len
# After that is data buffer
# After that is CRC 
# After that is 2 end bytes
# Whole string is packed in one list of data

def Pack_data(Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out):

    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52 #1
    Position = [Position_out[0],Position_out[1],Position_out[2],Position_out[3],Position_out[4],Position_out[5]]  #18
    Speed = [Speed_out[0], Speed_out[1], Speed_out[2], Speed_out[3], Speed_out[4], Speed_out[5],] #18
    Command = Command_out.value#1
    Affected_joint = Affected_joint_out
    InOut = InOut_out #1
    Timeout = Timeout_out.value #1
    Gripper_data = Gripper_data_out #9
    CRC_byte = 228 #1
    # End bytes = 2


    test_list = []
    #print(test_list)

    #x = bytes(start_bytes)
    test_list.append((start_bytes))
    
    test_list.append(bytes([len]))


    # Position data
    for i in range(6):
        position_split = Split_2_3_bytes(Position[i])
        test_list.append(position_split[1:4])

    # Speed data
    for i in range(6):
        speed_split = Split_2_3_bytes(Speed[i])
        test_list.append(speed_split[1:4])

    # Command data
    test_list.append(bytes([Command]))

    # Affected joint data
    Affected_list = Fuse_bitfield_2_bytearray(Affected_joint[:])
    test_list.append(Affected_list)

    # Inputs outputs data
    InOut_list = Fuse_bitfield_2_bytearray(InOut[:])
    test_list.append(InOut_list)

    # Timeout data
    test_list.append(bytes([Timeout]))

    # Gripper position
    Gripper_position = Split_2_3_bytes(Gripper_data[0])
    test_list.append(Gripper_position[2:4])

    # Gripper speed
    Gripper_speed = Split_2_3_bytes(Gripper_data[1])
    test_list.append(Gripper_speed[2:4])

    # Gripper current
    Gripper_current = Split_2_3_bytes(Gripper_data[2])
    test_list.append(Gripper_current[2:4])  

    # Gripper command
    test_list.append(bytes([Gripper_data[3]]))
    # Gripper mode
    test_list.append(bytes([Gripper_data[4]]))
    # Gripper ID
    test_list.append(bytes([Gripper_data[5]]))
 
    # CRC byte
    test_list.append(bytes([CRC_byte]))

    # END bytes
    test_list.append((end_bytes))
    
    #print(test_list)
    return test_list


# Data we send to the robot for testing
def Pack_data_test():

    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52 #1
    Position = [255,255,255,255,255,255]  #18
    Speed = [255,255,255,255,255,255]  #18
    Command = 123 #1 
    Affected_joint = [1,1,1,1,1,1,1,1] #1
    InOut = [0,0,0,0,0,0,0,0] #1
    Timeout = 247 #1
    Gripper_data = [-222,-223,-224,225,226,123]  #9
    CRC_byte = 228 #1
    # End bytes = 2


    test_list = []
    #print(test_list)

    #x = bytes(start_bytes)
    test_list.append((start_bytes))
    
    test_list.append(bytes([len]))

    # Position data
    for i in range(6):
        position_split = Split_2_3_bytes(Position[i])
        test_list.append(position_split[1:4])

    # Speed data
    for i in range(6):
        speed_split = Split_2_3_bytes(Speed[i])
        test_list.append(speed_split[1:4])

    # Command data
    test_list.append(bytes([Command]))

    # Affected joint data
    Affected_list = Fuse_bitfield_2_bytearray(Affected_joint)
    test_list.append(Affected_list)

    # Inputs outputs data
    InOut_list = Fuse_bitfield_2_bytearray(InOut)
    test_list.append(InOut_list)

    # Timeout data
    test_list.append(bytes([Timeout]))

    # Gripper position
    Gripper_position = Split_2_3_bytes(Gripper_data[0])
    test_list.append(Gripper_position[2:4])

    # Gripper speed
    Gripper_speed = Split_2_3_bytes(Gripper_data[1])
    test_list.append(Gripper_speed[2:4])

    # Gripper current
    Gripper_current = Split_2_3_bytes(Gripper_data[2])
    test_list.append(Gripper_current[2:4])  

    # Gripper command
    test_list.append(bytes([Gripper_data[3]]))
    # Gripper mode
    test_list.append(bytes([Gripper_data[4]]))
    # Gripper ID
    test_list.append(bytes([Gripper_data[5]]))
 
    # CRC byte
    test_list.append(bytes([CRC_byte]))

    # END bytes
    test_list.append((end_bytes))
    
    #print(test_list)
    return test_list

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
