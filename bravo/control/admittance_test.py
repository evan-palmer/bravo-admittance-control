import kinpy as kp
import numpy as np

bravo = kp.build_serial_chain_from_urdf(
    open(
        "C:/Users/marcu/OneDrive/Documents/GitHub/bravo-admittance-control/bravo/urdf/bravo7.urdf"
    ).read(),
    "ee_link",
)
jc = bravo.jacobian(
    [0.0] * 9
)  # 6 body joints, 1 prismatic joint for gripper and 2 revolute joints for jaw joints

## Getting joint velocities from jacobian with input linear ee velocities
# end_eff_vel = np.zeros([6, 1])
# end_eff_vel[0] = 0.1
# joint_vel = np.matmul(np.linalg.pinv(jc), end_eff_vel)
# print(joint_vel)

# Select which joints to set as active
selection_mat = np.zeros([6, 1])
selection_mat[5] = 1
# WHERE SHOULD WE APPLY THIS INTO OUR FORMULATION

# Body parameters
Md = np.diag([1, 1, 1, 1, 1, 1])
Kp = np.diag([0, 0, 0, 0, 0, 10])
Kd = np.diag([0, 0, 0, 0, 0, 10])
Kf = np.diag([0, 0, 0, 0, 0, 1])

# Initial Conditions
x_e = np.array([0, 0, 0, 0, 0, 0.5])  # Current ee-position
x_d = np.array([0, 0, 0, 0, 0, 0])  # Desired ee-position
f_e = np.array([0, 0, 0, 0, 0, 1])  # Current ee-force
f_d = np.array([0, 0, 0, 0, 0, 0.1])  # Desired ee-force

# Equation to follow
# y = J(q)^-1 * Md^-1 * (-Kd * x_e_dot + Kp * (x_error - f_error) - Md * J_dot(q, q_dot) * q_dot)
#     [--- outer ---] * [------------------------------ inner ----------------------------------]
# Currently ignoring the second order jacobian term since I do not know how to deal with it

# Controller formulation
x_error = x_d - x_e
f_error = f_d - f_e
x_f = np.matmul(Kf, f_error)
prop_error = x_error + x_f

outer = np.matmul(np.linalg.pinv(jc), np.linalg.inv(Md))
inner = np.matmul(-Kd, x_e) + np.matmul(Kp, prop_error)

output = np.matmul(outer, inner)
print(output)
