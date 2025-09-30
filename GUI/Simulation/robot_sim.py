#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np
from math import pi

# ---------------------------
# 1) 几何参数（单位：米）
# ---------------------------
a1 = 110.50 / 1000
a2 = 23.42  / 1000
a3 = 180.00 / 1000
a4 = 43.50  / 1000
a5 = 176.35 / 1000
a6 = 110.0  / 1000   # 法兰到 TCP 的 +x 偏置（放到 tool）
a7 = 45.25  / 1000   # 法兰到 TCP 的 -z 偏置（放到 tool）

alpha_DH = [-pi/2,  pi,  pi/2, -pi/2,  pi/2,  pi]

# ---------------------------
# 2) 关节上下限（度 → 弧度）
#    形状需为 (2,6) 的 numpy 数组
# ---------------------------
Joint_limits_degree = [
    [-123.046875, 123.046875],   # J1
    [-145.0088  ,  -3.375   ],   # J2
    [ 107.866   , 287.8675  ],   # J3
    [-105.46975 , 105.46975 ],   # J4
    [ -90.0     ,  90.0     ],   # J5
    [   0.0     , 360.0     ],   # J6
]
Joint_limits_radian = [np.radians(lim) for lim in Joint_limits_degree]
qlim = np.array(Joint_limits_radian).T   # (2,6)

# ---------------------------
# 3) 建模（末端几何偏置入 tool）
# ---------------------------
robot = DHRobot(
    [
        RevoluteDH(d=a1,  a=a2,  alpha=alpha_DH[0]),   # J1
        RevoluteDH(d=0.0, a=a3,  alpha=alpha_DH[1]),   # J2
        RevoluteDH(d=0.0, a=-a4, alpha=alpha_DH[2]),   # J3
        RevoluteDH(d=-a5, a=0.0, alpha=alpha_DH[3]),   # J4
        RevoluteDH(d=0.0, a=0.0, alpha=alpha_DH[4]),   # J5
        RevoluteDH(d=-a6, a=0.0, alpha=alpha_DH[5]),   # J6
    ],
    name="PAROL6",
)



# ⚠️ 你的 rtb 版本：qlim 只读，不能直接赋值
#    用 __dict__ 绕过 property 限制
robot.__dict__["qlim"] = qlim

# ---------------------------
# 4) 站姿/仿真
# ---------------------------
q_standby_deg = [-17.763, -111.524, 123.046, 21.108, 35.62, 249.936]

q_standby = np.radians(q_standby_deg)

print("=== FK at standby (TCP pose) ===")
T = robot.fkine(q_standby)
print(T)

# 打开交互教示窗口（带滑杆、连杆骨架、TCP）
# 备注：某些环境需 plt.ion() / 后端支持
robot.teach(q_standby)

# ---------------------------
# 5) 可选：IK 自检（用 FK 的结果回逆解）
# ---------------------------
sol = robot.ikine_LMS(T, q0=q_standby, ilimit=20)
print("IK success:", getattr(sol, "success", False))
if hasattr(sol, "q"):
    err = np.linalg.norm(sol.q - q_standby)
    print("||q* - q|| =", err)


