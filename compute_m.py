#Defining 2x2 Mass matrix based on active joints q1 and q2
#Paper: http://www.mnrlab.com/uploads/7/3/8/3/73833313/modeling-of-pantograph.pdf
import numpy as np

def compute_M(q,x,y,J,L1,L2,D,m1,m2):
  #x,y are end effector position, q is two active joint angles [q1,q2]
  #J is 2x2 jacobian at current configuration, (L1, m1) and (L2,m2) are first/second link lengths and masses
  #D is the base link length

  #Extracting Link geometry
  #Using convention from the paper, q1 and q4 are active joints
  q1 = q[0]
  q4 = q[1]

  #Vectors from each elbow joint to end-effector
  dx1 = x-L1*np.cos(q1)
  dx2 = x-D-L1*np.cos(q4)
  dy1 = y-L1*np.sin(q1)
  dy2 = y-L1*np.sin(q4)

  #Defining P matrix, converts from active joint velocities to all joint velocities
  P21 = (1/L2**2)*(dx1*(J[1,0]-L1*np.cos(q1))-dy1*(J[0,0]+L1*np.sin(q1)))
  P22 = (1/L2**2)*(dx1*J[1,1]-dy1*J[0,1])
  P31 = (1/L2**2)*(dx2*J[1,0]-dy2*J[0,0])
  P32 = (1/L2**2)*(dx2*(J[1,1]-L1*np.cos(q4))-dy2*(J[0,1]+L1*np.sin(q4)))

  #Assembling P matrix, 4x2
  P = np.array([[1,0],[P21,P22],[P31,P32],[0,1]])

  #Deriving passive joint angles from active joint angles
  q2 = np.arctan2(y-L1*np.sin(q1),x-L1*np.cos(q1))
  q3 = np.arctan2(y-L1*np.sin(q4),x-L1*np.cos(q4))

  #Defining 4x4 Mass Matrix from paper
  M4 = np.array([
        [((m1/3) + m2)*L1**2,0.25*m2*L1*L2*np.cos(q1-q2),0,0],
        [0.25*m2*L1*L2*np.cos(q1-q2),(m2/3)*L2**2,0,0],
        [0,0,(m2/3)*L2**2,0.25*m2*L2*L1*np.cos(q3-q4) ],
        [0,0,0.25*m2*L2*L1*np.cos(q3-q4), ((m1/3) + m2)*L1**2]
    ])

  #Converting 4x4 Mass Matrix in to 2x2 Matrix
  M2 = np.transpose(P)@M4@P #[2x4]*[4x4]*[4x2]
  return M2