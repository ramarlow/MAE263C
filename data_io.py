'''
utility/helper classes for getting data into and out of the script
inputs:
 - nothing (i couldn't get the serial reading to behave nicely)
outputs:
 - UDP publishing for plotting
'''

import socket, time
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
