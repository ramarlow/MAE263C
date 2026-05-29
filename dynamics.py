'''
forward dynamics math for finding end effector forces given load cell shear forces
'''

import numpy as np
from kinematics import FK

def F_ee(f, q):
    '''
    manipulator end effector forces based on load cell readings
    '''
    f1 = f[0]; f2 = f[1];
    q1 = q[0]; q2 = q[1];

    Pe, P2, P4, Ph = FK(q1,q2)

    u_12 = np.array([np.cos(q1),np.sin(q1)])
    t_2 = np.array([-u_12[0],u_12[1]])
    u_54 = np.array([np.cos(q2),np.sin(q2)])
    t_4 = np.array([u_54[0],-u_54[1]])

    f_res = f1*t_2 + f2*t_4
    f_res[0] *= -2. # scale to deal with isotropy and flipping :(
    f_res /= 600    # arbitrary calibration factor

    return f_res