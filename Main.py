from dynamixel_sdk import *
from forward_kinematics import forward_kinematics
from kinematics import Jacobian, FK
from operation_space_PID import PDController
import numpy as np
import dynamics, data_io
import serial, serial.threaded

class StoreLoads(serial.threaded.LineReader):
    def handle_line(self, line):
        global loads
        loads = np.array([float(load) for load in line.split('\t')])
        # print(loads)

data_out = data_io.UDP_Client()
PORT_DYN = "COM10"
PORT_ARD = "COM13"
BAUD = 57600
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

controller = PDController(setpoint=[.1, 0.350], 
                          kp=50.0, 
                          kd=0.,#50.0, 
                          dt=0.01)

for id in IDS:
    packet.write1ByteTxRx(port, id, 11, 16)  # PWM mode
    packet.write1ByteTxRx(port, id, 64, 1)   # torque enable

def ticks_to_deg(ticks):
    return ticks * 2 * np.pi / 4096.0

D = .200
L1 = .220
L2 = .250

try:
    while 1:
        for id in IDS:
            pos, _, _ = packet.read4ByteTxRx(port, id, 132)
            # print(pos)
            if pos > 0x7FFFFFFF:
                pos -= 0x100000000

            angle = ticks_to_deg(pos)
            if id == 6:
                theta2 = np.pi-angle
            else:
                theta1 = 2*np.pi-angle

        pos = forward_kinematics(theta1, theta2, L1, L2, D)
        # eepos2, P2, P4, Ph = FK(theta1,theta2)

        # Each timestep, call:
        force = controller.update(x=pos[0], y=pos[1])
        # returns np.array([Fx, Fy])
        
        tau = Jacobian(theta1,theta2).T@force.T

        K_PWM = 0.8

        packet.write2ByteTxRx(port, 5, 100, int(tau[0]*-K_PWM) & 0xFFFF)
        packet.write2ByteTxRx(port, 6, 100, int(tau[1]*-K_PWM) & 0xFFFF)

        print(f'force:{force}, torques:{tau}, PWMs:{tau*K_PWM}')
        data_out.send({
                'q1':float(theta1),'q2':float(theta2),      # joint angles
                'ex':float(-pos[0]),'ey':float(pos[1]),     # fk ee pos
                'fx':float(-force[0]),'fy':float(force[1]), # desired ee force
                'tau1':float(tau[0]),'tau2':float(tau[1]),  # desired joint torques
                'f1':float(loads[0]),'f2':float(loads[1]),  # load cell readings
                })
finally:
    for id in IDS:
        packet.write2ByteTxRx(port, id, 100, 0)
        packet.write1ByteTxRx(port, id, 64, 0)
    port.closePort()
