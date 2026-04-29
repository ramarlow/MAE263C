from forward_kinematics import forward_kinematics
from operation_space_PID import PDController
import numpy as np

theta1 = np.pi/2
theta2 = np.pi/2

D = .200
L1 = .220
L2 = .250

pos = forward_kinematics(theta1, theta2, L1, L2, D)
controller = PDController(setpoint=[.1, 0.450], kp=50.0, kd=10.0, dt=0.01)

# Each timestep, call:
force = controller.update(x=pos[0], y=pos[1])
# returns np.array([Fx, Fy])
print(force)

