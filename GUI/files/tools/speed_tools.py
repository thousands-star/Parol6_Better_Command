import numpy as np
import tools.PAROL6_ROBOT as R

def percent_to_joint_speed(joint_id: int, percent: float, direction: int) -> int:
    min_speed = R.Joint_min_jog_speed[joint_id]
    max_speed = R.Joint_max_jog_speed[joint_id]
    abs_speed = int(np.interp(percent, [0, 100], [min_speed, max_speed]))
    return direction * abs_speed
