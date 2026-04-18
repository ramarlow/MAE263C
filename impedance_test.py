import time
import socket
from json import dumps
from motor_utils import *

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


port = "COM10"
motors = MotorArr(port, 57600, [1])
motors.torque_disable()
motors.set_mode('vel')
motors.torque_enable()

data_out = UDP_Client()
err_hist = np.zeros(30)
err_idx = 0
try:
    while 1:
        load_torques = motors.get_loads()
        
        err_hist[err_idx]=load_torques[0] # moving avg filter
        err_idx+=1
        err_idx%=len(err_hist)
        avg_load=np.mean(err_hist)

        vel_gain=-10000
        motors.set_velocities([avg_load*vel_gain])
        data_out.send({'load':load_torques[0],'load_filt':avg_load,'vel':avg_load*vel_gain})
        time.sleep(0.0001)
finally:
    motors.torque_disable()