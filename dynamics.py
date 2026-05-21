'''
forward dynamics math for finding end effector forces given load cell shear forces
'''

import numpy as np
from kinematics import FK

def F_ee(f1, f2=None):
    '''
    manipulator end effector forces based on load cell readings
    '''
    return 