# import pybullet as p
from math import pi

import kinpy as kp
import numpy as np

# physicsClient = p.connect(p.DIRECT)    # Non-graphical version

##### Load files #####
# kinpy version - to perform kinematic calcs
bravo_kin = kp.build_serial_chain_from_urdf(
    open(
        "C:/Users/marcu/OneDrive/Documents/GitHub/bravo-admittance-control/bravo/config/urdf/bravo7.urdf"
    ).read(),
    "ee_link",
)

# pybullet version - to perform mass matrix calcs
# p.setGravity(0, 0, -9.8)
# start_pos = [0, 0, 0]
# start_orientation = p.getQuaternionFromEuler([0, 0, 0])
# bravo_bullet = p.loadURDF("C:/Users/marcu/OneDrive/Documents/GitHub/bravo-admittance-control/bravo/urdf/bravo7_obj.urdf", start_pos, start_orientation)

##### Selected Parameters #####
selection_mat = np.diag(
    [0, 0, 0, 0, 0, 1]
)  # Selection matrix - ensures that the admittance controller only generates motion in the specified directions
Md = np.diag([0, 0, 0, 0, 0, 10])  # Mass matrix
Kp = np.diag([0, 0, 0, 0, 0, 10])  # Proportional gain
Kd = np.diag(
    [0, 0, 0, 0, 0, 10]
)  # Derivative gain (not used in our formulation since we're doing parallel force/position)
Kf = np.diag([0, 0, 0, 0, 0, 1])  # Force gain
x_d = np.array([0, 0, 0, 0, 0, 0])  # Desired ee-position
f_d = np.array([0, 0, 0, 0, 0, 0.1])  # Desired ee-force

##### Sensor Readings #####
joint_configs = [
    0,
    pi / 2,
    pi,
    0,
    pi,
    0,
    0,
    0,
    0,
]  # 6 body joints, 1 prismatic joint for gripper and 2 revolute joints for jaw joints
# Md = np.array(p.calculateMassMatrix(bravo_bullet, joint_configs)) # TODO: Need to verify that the joint configurations between pybullet and kinpy line up
x_e = np.array([0, 0, 0, 0, 0, 0.5])  # Current ee-position - Incoming sensor reading
x_e_dot = np.array(
    [0, 0, 0, 0, 0, 0.1]
)  # Current ee-velocity - Incoming sensor reading
f_e = np.array([0, 0, 0, 0, 0, 1])  # Current ee-force - Incoming sensor reading

##### Controller Formulation #####
# Equation to follow
# y = J(q)^-1 * Md^-1 * (-Kd * x_e_dot + Kp * (x_error - f_error) - Md * J_dot(q, q_dot) * q_dot)
#     [--- outer ---] * [------------------------------ inner ----------------------------------]
# Currently ignoring the second order jacobian term since I do not know how to deal with it

jc = bravo_kin.jacobian(joint_configs)  # Jacobian based on incoming joint configs

# # Equation in steps
# x_error = x_d - x_e
# f_error = f_d - selection_mat @ f_e
# x_f = Kf @ f_error
# prop_error = x_error + x_f
# outer = np.linalg.pinv(jc) @ Md
# inner = -Kd @ x_e_dot + Kp @ prop_error
# output = outer @ inner
# print(output)

# Equation in one foul swoop
joint_vel = (
    np.linalg.pinv(jc)
    @ np.linalg.pinv(Md)
    @ (-Kd @ x_e_dot - Kp @ (x_d - x_e + Kf @ (f_d - selection_mat @ f_e)))
)
print(joint_vel)
# TODO: Need to verify matrix dimensions
# J^-1 = 9x6
# M^-1 = 9x9
# (..) = 6x1
