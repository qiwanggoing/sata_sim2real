# import pinocchio as pin
# from pinocchio import JointModelFreeFlyer # type: ignore
# from pinocchio.robot_wrapper import RobotWrapper
# import numpy as np

# # 加载模型
# model = mujoco.MjModel.from_xml_path("/home/song/mujoco-3.3.0-linux-x86_64/mujoco-3.3.0/model/go2/go2.xml")
# data = mujoco.MjData(model)

# # 设定一个姿态 q
# data.qpos[:] = [ 0.01622374, -0.05040711,  0.30026103,  0.70342921, -0.01303133,  0.02349065,
#   0.7102575,  -0.22412577,  0.82808785, -1.62305806,  0.20543067,  0.74320843,
#  -1.60106509, -0.11607908,  1.05651889, -1.55083853,  0.19652592,  0.94320221,
#  -1.36076884]

# mujoco.mj_forward(model, data)

# # 清零加速度和速度
# data.qvel[:] = 0
# data.qacc[:] = 0
# mujoco.mj_inverse(model, data)
# # print("Required torques:", data.qfrc_inverse)

# # # 清空控制输入
# data.ctrl[:] = 0

# # 模拟重力效应
# mujoco.mj_rnePostConstraint(model, data)

# # data.qfrc_bias 就是广义力，包括重力 + 离心/科氏项
# # 若 v = 0（静止），那么它就等于 g(q)
# g_q = data.qfrc_inverse.copy()

# print("g(q) =", g_q)
