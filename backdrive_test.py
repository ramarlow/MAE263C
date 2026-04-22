import time
import socket
from json import dumps
from motor_utils import *
import serial

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


port_u2d2 = "COM10"
try:
    motors = MotorArr(port_u2d2, 57600, [1, 2])
except: pass

motors.torque_disable()
motors.set_mode('pwm')
motors.torque_enable()

data_out = UDP_Client()
err_hist = np.zeros(1) # buffer of past load values
err_idx = 0
load_cell = 0
PWM_gain=-30000

try:
    while 1:
        load_torques = motors.get_loads()
        
        err_hist[err_idx]=load_torques[0] # moving avg filter
        err_idx+=1
        err_idx%=len(err_hist)
        avg_load=np.mean(err_hist)

        PWM_target = avg_load*PWM_gain
        motors.apply_write_table("Goal_PWM",[0,PWM_target])
        data_out.send({
            'load0':load_torques[0],
            'load1':load_torques[1],
            'PWM':PWM_target})
        time.sleep(0.0001)
finally:
    motors.torque_disable()