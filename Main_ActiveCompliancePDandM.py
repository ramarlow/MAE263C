from dynamixel_sdk import *
from forward_kinematics import forward_kinematics
from kinematics import Jacobian, FK
from operation_space_PID import PDController
from potential_field import CircleField
import numpy as np
import dynamics, data_io
import serial, serial.threaded
from compute_m import compute_M

class StoreLoads(serial.threaded.LineReader):
    def handle_line(self, line):
        global loads
        try:
            loads = np.array([float(load) for load in line.split('\t')])
        except:
            print('receiving error')
        # print(loads)

data_out = data_io.UDP_Client()
PORT_DYN = "COM10"
PORT_ARD = "COM13"
BAUD = 1e6
IDS = [5, 6]
loads = np.zeros((2,))

def ticks_to_deg(ticks):
    return ticks * 2 * np.pi / 4096.0

try:
    port = PortHandler(PORT_DYN)
    packet = PacketHandler(2.0)
    port.openPort()
    port.setBaudRate(BAUD)
except:
    print('dynamixels not connected, can\'t really run')
    raise NotImplementedError('maybe we can find some way to run in simulation instead? idk')

try:
    arduino = serial.Serial(PORT_ARD, 57600, timeout=0.01, write_timeout=0.1)
    reader = serial.threaded.ReaderThread(arduino, StoreLoads)
    reader.start()
except:
    print('no arduino connected, running without load cells')
    arduino = None

controller = CircleField(center=[0.10, 0.375], radius=0.025, k=2) #k is position gain

_N_CIRC = 60
_circ_angles = np.linspace(0, 2*np.pi, _N_CIRC, endpoint=False)
_circ_xs = controller.center[0] + controller.r * np.cos(_circ_angles)
_circ_ys = controller.center[1] + controller.r * np.sin(_circ_angles)
_circ_idx = 0

for id in IDS:
    packet.write1ByteTxRx(port, id, 11, 16)  # PWM mode
    packet.write1ByteTxRx(port, id, 64, 1)   # torque enable

D = .200
L1 = .220
L2 = .250
m1 = 50/1000 #kg
m2 = 58.25/1000 #kg

K_D = np.diag([2,2]) #Cartesian Velocity Damping
D_on = True #Boolean for derivative term on/off
M_on = False #Boolean for mass matrix on/off


try:
    while 1:
        read_start = time
        for id in IDS:
            pos, _, _ = packet.read4ByteTxRx(port, id, 132)
            vel, _, _ = packet.read4ByteTxRx(port, id, 128)  # Velocity
            
            # print(pos)
            if pos > 0x7FFFFFFF:
                pos -= 0x100000000

            if vel > 0x7FFFFFFF: #Accounting for negative velocity
                vel -= 0x100000000

            angle = ticks_to_deg(pos)
            angle_vel = ticks_to_deg(vel)/60 #ticks/min -> rad/min ->rad/s

            if id == 6:
                theta2 = -angle+2*np.pi
                theta2_dot = -angle_vel
            else:
                theta1 = -angle+2*np.pi
                theta1_dot = -angle_vel


        pos = forward_kinematics(theta1, theta2, L1, L2, D)
        # eepos2, P2, P4, Ph = FK(theta1,theta2)

        # Each timestep, call:
        force = controller.update(x=pos[0], y=pos[1])
        # returns np.array([Fx, Fy])

        #Defining array for joint angular velocities
        theta_dot = np.array([theta1_dot,theta2_dot])

        #Calculating Jacobian
        J = Jacobian(theta1,theta2)

        #Calculating control torque
        if D_on:
            if M_on:
                #Calculating M matrix
                M = compute_M(np.array([theta1, theta2]),pos[0],pos[1],J,L1,L2,D,m1,m2)
                M_inv = np.linalg.inv(M)
                Lambda = np.linalg.inv(J @ M_inv @ J.T+ 0.001 * np.eye(2)) #Task space mass matrix, second term avoids singularities
                tau = J.T @ Lambda @ (force.T - K_D @ J @ theta_dot.T)
            else:
                tau = J.T@(force.T-K_D@J@theta_dot.T)
        else:
            if M_on:
                #Calculating M matrix
                M = compute_M(np.array([theta1, theta2]),pos[0],pos[1],J,L1,L2,D,m1,m2)
                M_inv = np.linalg.inv(M)
                Lambda = np.linalg.inv(J @ M_inv @ J.T+ 0.001 * np.eye(2)) #Task space mass matrix
                tau = J.T @ Lambda @force.T
            else:
                tau = J.T@force.T


        f_cells = -dynamics.F_ee(loads, np.array([theta1,theta2]))

        K_PWM = 0.8


        packet.write2ByteTxRx(port, 5, 100, int(tau[0]*-K_PWM) & 0xFFFF)
        packet.write2ByteTxRx(port, 6, 100, int(tau[1]*-K_PWM) & 0xFFFF)

        # print(f'force:{force}, torques:{tau}, PWMs:{tau*K_PWM}')
        data_out.send({
                'q1':float(theta1),'q2':float(theta2),              # joint angles
                'ex':float(-pos[0]),'ey':float(pos[1]),             # fk ee pos
                'fx_des':float(-force[0]),'fy_des':float(force[1]), # desired ee force
                'tau1':float(tau[0]),'tau2':float(tau[1]),          # desired joint torques
                'f1':float(loads[0]),'f2':float(loads[1]),          # load cell readings
                'fx_cell':float(f_cells[0]),'fy_cell':float(f_cells[1]), # transformed load cell readings
                'circle_x':-float(_circ_xs[_circ_idx % _N_CIRC]),
                'circle_y':float(_circ_ys[_circ_idx % _N_CIRC]),
                })
        _circ_idx += 1
finally:
    for id in IDS:
        packet.write2ByteTxRx(port, id, 100, 0)
        packet.write1ByteTxRx(port, id, 64, 0)
    port.closePort()
