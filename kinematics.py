'''
forward/inverse/differential kinematics derived in:

[1] G. Campion, Q. Wang, and V. Hayward, “The Pantograph Mk-II: a haptic instrument,” in 2005 IEEE/RSJ International Conference on Intelligent Robots and Systems, Aug. 2005, pp. 193-198. doi: 10.1109/IROS.2005.1545066.

with some minor modifications:
- assumed link length symmetry (may change later)
- shifted base frame origin to be at the middle of the base link
'''

import numpy as np

link = {
    'base':200,
    'prox':220,
    'dist':250,
        }

a = [0,     # aliases for link lengths so they match the paper, which uses 1 indexing
     link['prox'],
     link['dist'],
     link['dist'],
     link['prox'],
     link['base']]

def FK(q, q2=None):
    '''
    manipulator forward kinematics based on eqns (1)-(7)
    params: either 2 angle arguments or a vector of 2 angles (in radians)
    returns: 
    - position vector of endpoint Pe 
    - P2 and P4 (the "elbows")
    - Ph
    '''
    if q2 == None: # try to tolerate both 2 argument and 1 vector argument
        q1 = q[0]
        q2 = q[1]
    else:
        q1 = q

    P2 = np.array([a[1]*np.cos(q1)-a[5]/2,          # (1)
                   a[1]*np.sin(q1)]).T
    P4 = np.array([a[4]*np.cos(q2)+a[5]/2,          # (2)
                   a[4]*np.sin(q2)]).T
    
    P4_P2 = np.linalg.norm(P4-P2)
    P2_Ph = (a[2]**2-a[3]**2+P4_P2**2)/(2*P4_P2)    # (3)
    Ph = P2 + P2_Ph/P4_P2 * (P4-P2)                 # (4)
    P3_Ph = np.sqrt(a[2]**2-P2_Ph**2)               # (5)

    xe = Ph[0] - P3_Ph * (P4[1]-P2[1])/P4_P2        # (6)
    ye = Ph[1] + P3_Ph * (P4[0]-P2[0])/P4_P2        # (7)

    return (np.array([xe,ye]).T, P2, P4, Ph)

def IK(Pe, ye=None):
    '''
    manipulator inverse kinematics based on eqns (8)-(12) 
    params: position vector of end effector
    returns: vector of joint angles to achieve the desired endpoint (in radians)
    '''
    if ye == None: # try to tolerate both 2 argument and 1 vector argument
        xe = Pe[0]
        ye = Pe[1]
    else:
        xe = Pe

    P1_Pe = np.linalg.norm(Pe-np.array([a[5]/2,0]).T)
    P5_Pe = np.linalg.norm(Pe-np.array([-a[5]/2,0]).T)
    alpha1 = np.arccos(                                     # (9)
        (a[1]**2-a[2]**2+P1_Pe**2) / (2*a[1]*P1_Pe)   )
    beta1 = np.arctan2(ye, a[5]/2-xe)                       # (10)

    alpha2 = np.arccos(                                     # (11)
        (a[4]**2-a[3]**2+P5_Pe**2) / (2*a[4]*P5_Pe)   )
    beta2 = np.arctan2(ye, a[5]/2+xe)                       # (12)

    q2 = np.pi - alpha1 - beta1                             # (8)
    q1 = alpha2 + beta2

    return np.array([q1,q2]).T

def Jacobian(q, q2=None):
    '''
    pose-dependant geometric jacobian based on eqns (13)-(22)
    params: either 2 angle arguments or a vector of 2 angles (in radians)
    returns: [2x2] geometric jacobian
    '''
    if q2 == None: # try to tolerate both 2 argument and 1 vector argument
        q1 = q[0]
        q2 = q[1]
    else:
        q1 = q


    Pe, P2, P4, Ph = FK(q1,q2)
    d = np.linalg.norm(P2-P4)
    b = np.linalg.norm(P2-Ph)
    h = np.linalg.norm(Pe-Ph)

    #q1, q2 = q2, q1

    d1dx2 = -a[1]*np.sin(q1)             # (14)
    d1dy2 = a[1]*np.cos(q1)

    d5dx4 = -a[4]*np.sin(q2)             # (15)
    d5dy4 = a[4]*np.cos(q2)

    d1dy4 = 0                           # (16)
    d1dx4 = 0
    d5dy2 = 0
    d5dx2 = 0

    x4_x2 = P4[0]-P2[0]
    y4_y2 = P4[1]-P2[1]

    d1d = (x4_x2*(d1dx4 - d1dx2) + y4_y2*(d1dy4 - d1dy2))/d     # (17)
    d5d = (x4_x2*(d5dx4 - d5dx2) + y4_y2*(d5dy4 - d5dy2))/d

    d1b = d1d*(1 - (a[2]**2 - a[3]**2 + d**2)/(2*d**2))       # (18)
    d5b = d5d*(1 - (a[2]**2 - a[3]**2 + d**2)/(2*d**2))

    d1h = -b*d1b/h                                              # (16)
    d5h = -b*d5b/h

    d1dyh = d1dy2 + (d1b*d - d1d*b)/d**2 * y4_y2 + b/d*(d1dy4 - d1dy2) # (19)
    d5dyh = d5dy2 + (d5b*d - d5d*b)/d**2 * y4_y2 + b/d*(d5dy4 - d5dy2) 

    d1dxh = d1dx2 + (d1b*d - d1d*b)/d**2 * x4_x2 + b/d*(d1dx4 - d1dx2) # (20)
    d5dxh = d5dx2 + (d5b*d - d5d*b)/d**2 * x4_x2 + b/d*(d5dx4 - d5dx2)

    d1dy3 = d1dyh - h/d*(d1dx4 - d1dx2) - (d1h*d - d1d*h)/d**2 * x4_x2 # (21)
    d5dy3 = d5dyh - h/d*(d5dx4 - d5dx2) - (d5h*d - d5d*h)/d**2 * x4_x2

    d1dx3 = d1dxh + h/d*(d1dy4 - d1dy2) + (d1h*d - d1d*h)/d**2 * y4_y2 # (22)
    d5dx3 = d5dxh + h/d*(d5dy4 - d5dy2) + (d5h*d - d5d*h)/d**2 * y4_y2

    return np.array([                                           # (13)
        [d5dx3, d1dx3],
        [d5dy3, d1dy3]])


def IK_numerical(Pe_des,q_guess=np.array([np.pi/2,np.pi/2]).T,tol=1e-3,maxiters=100):
    '''
    ik using newton-raphson iteration to converge toward a solution
    '''

    for i in range(maxiters):   # newton-raphson iteration to find a solution
        # print(q_guess)
        Pe_curr = FK(q_guess)[0] # fk to find end position given guess
        # print(Pe_curr)
        err = Pe_curr - Pe_des # positional error
        # print(err)

        J = Jacobian(q_guess) # geometric jacobian of manipulator in current configuration

        try:
            dq = 0.2 * np.linalg.lstsq(J,err,rcond=None)[0] # scale solution to mitigate overshooting
        except np.linalg.LinAlgError as e:
            # print(e)
            raise np.linalg.LinAlgError(f'Singularity encountered while solving at iteration {i}')

        q_guess = q_guess - dq # adjust guess with correction factor

        if np.linalg.norm(err) < tol: # solution converged to within tolerance
            return q_guess
        
    raise RuntimeError('solution failed to converge :(') # max iters reached, complain


# testing

'''
eepos = FK(np.pi/2, np.pi/2)[0]
print(eepos)
pose_IK = IK(eepos)
print(pose_IK)
pose_IK_numerical = IK_numerical(eepos, np.array([1.6,1.6]).T, 1e-3, 300)

print(Jacobian(pose_IK))
print(eepos, pose_IK, pose_IK_numerical)
'''