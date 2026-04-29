
import numpy as np

#D = 200
#L1 = 220
#L2 = 250
#theta1 = np.pi/2
#theta2 = np.pi/2

def forward_kinematics(theta1, theta2, L1, L2, D):
    """
    Compute the XY position of the end effector for a two-arm delta-style mechanism.

    Parameters:
        theta1, theta2 : joint angles (radians)
        L1             : proximal link length
        L2             : distal link length (shared / rod length)
        D              : x-axis offset of the second arm's base

    Returns:
        np.array([x, y]) : end effector position
    """
    # Elbow positions
    E1 = np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
    E2 = np.array([D + L1 * np.cos(theta2), L1 * np.sin(theta2)])

    # Midpoint and distance between elbows
    d = np.linalg.norm(E2 - E1)
    M = (E1 + E2) / 2

    # Height of the intersection point above the E1-E2 line
    h = np.sqrt(L2 ** 2 - (d / 2) ** 2)

    # Unit normal to the E1-E2 segment
    n = np.array([E1[1] - E2[1], E2[0] - E1[0]]) / d

    # End effector position
    fk = M + h * n

    return fk


