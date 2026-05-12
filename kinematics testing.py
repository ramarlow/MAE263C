from dynamixel_sdk import *
import numpy as np
from forward_kinematics import forward_kinematics
from kinematics import FK, IK, IK_numerical, Jacobian
import time
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

PORT = "COM10"
BAUD = 57600
IDS = [5, 6]

def ticks_to_deg(ticks):
    return ticks * 2 * np.pi / 4096.0

data_out = UDP_Client()
port = PortHandler(PORT)
packet = PacketHandler(2.0)

port.openPort()
port.setBaudRate(BAUD)

for id in IDS:
    packet.write1ByteTxRx(port, id, 11, 16)  # PWM mode
    packet.write1ByteTxRx(port, id, 64, 1)   # torque enable

q = np.array([0.,0.]).T

D = .200
L1 = .220
L2 = .250

try:
    while True:
        for idx, id in enumerate(IDS):
            pos, _, _ = packet.read4ByteTxRx(port, id, 132)
            # print(pos)
            if pos > 0x7FFFFFFF:
                pos -= 0x100000000

            angle = ticks_to_deg(pos)
            if idx == 0:
                q[idx] = angle-np.pi
            else:
                q[idx] = angle

        theta1 = np.pi-q[0]
        theta2 = np.pi-q[1]
        eepos = forward_kinematics(theta1, theta2, L1, L2, D)
        eepos2, P2, P4, Ph = FK(theta1,theta2)
        ikq = IK(eepos2)
        iknq = IK_numerical(eepos2)
        # eepos=(0,0)
        J = Jacobian(theta1,theta2)
        # print(J)
        data_out.send({
            'q1':float(theta1),'q2':float(theta2),
            'q1_ik':float(ikq[0]),'q2_ik':float(ikq[1]),
            'q1_ikn':float(iknq[0]),'q2_ikn':float(iknq[1]),
            'x':float(eepos[0]-0.1),'y':float(eepos[1]),
            'x_ee':float(eepos2[0]/1000),'y_ee':float(eepos2[1]/1000),
            'x_p2':float(P2[0]/1000),'y_p2':float(P2[1]/1000),
            'x_p4':float(P4[0]/1000),'y_p4':float(P4[1]/1000),
            'x_ph':float(Ph[0]/1000),'y_ph':float(Ph[1]/1000),
            'dexd1':float((J[0,0]+eepos2[0])/1e3),'deyd1':float((J[1,0]+eepos2[1])/1e3),
            'dexd2':float((J[0,1]+eepos2[0])/1e3),'deyd2':float((J[1,1]+eepos2[1])/1e3)})
except:
    pass