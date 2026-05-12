from dynamixel_sdk import *
from forward_kinematics import forward_kinematics
from kinematics import Jacobian
from operation_space_PID import PDController
import numpy as np
import socket
from json import dumps

class UDP_Client:
    def __init__(self, host: str = "127.0.0.1", port: int = 9870):
        self._UDP_addr = (host, port)
        self._UDP_sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    def send(self, data: dict):
        '''send data to UDP server, used for plotting with plotjuggler'''
        try:
            data["time"] = round(time.time(), 3)
            # print(data)
            self._UDP_sock.sendto(bytes(dumps(data),encoding='utf-8'), self._UDP_addr)
        except Exception as e:
            print(f'Send failed: {e}')

    def close(self):
        self._UDP_sock.close()

    def __del__(self):
        self.close()

data_out = UDP_Client()
PORT = "COM10"
BAUD = 57600
IDS = [5, 6]

def ticks_to_deg(ticks):
    return ticks * 2 * np.pi / 4096.0

port = PortHandler(PORT)
packet = PacketHandler(2.0)

port.openPort()
port.setBaudRate(BAUD)

controller = PDController(setpoint=[.1, 0.350], 
                          kp=50.0, 
                          kd=0,#50.0, 
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

        # Each timestep, call:
        force = controller.update(x=pos[0], y=pos[1])
        # returns np.array([Fx, Fy])
        

        tau = Jacobian(theta1,theta2).T@force.T

        K_PWM = -0.8

        packet.write2ByteTxRx(port, 5, 100, int(tau[0]*K_PWM) & 0xFFFF)
        packet.write2ByteTxRx(port, 6, 100, int(tau[1]*K_PWM) & 0xFFFF)

        print(f'force:{force}, torques:{tau}, PWMS:{tau*K_PWM}')
        data_out.send({
                'q1':float(theta1),'q2':float(theta2),
                'ex':float(pos[0]),'ey':float(pos[1]),
                'fx':float(force[0]),'fy':float(force[1]),
                'tau1':float(tau[0]),'tau2':float(tau[1])})
finally:
    for id in IDS:
        packet.write2ByteTxRx(port, id, 100, 0)
        packet.write1ByteTxRx(port, id, 64, 0)
    port.closePort()
