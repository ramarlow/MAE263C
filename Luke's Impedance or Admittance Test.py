from dynamixel_sdk import *
import time

PORT = "COM7"
BAUD = 57600
IDS = [5, 6]

K = 5.0  # PWM per degree, tune this
CENTERS_DEG = {5: 180.0, 6: 180.0}  # center angles in degrees

def ticks_to_deg(ticks):
    return ticks * 360.0 / 4096.0

port = PortHandler(PORT)
packet = PacketHandler(2.0)

port.openPort()
port.setBaudRate(BAUD)

for id in IDS:
    packet.write1ByteTxRx(port, id, 11, 16)  # PWM mode
    packet.write1ByteTxRx(port, id, 64, 1)   # torque enable

print("Running virtual spring. Ctrl+C to stop.")
try:
    while True:
        for id in IDS:
            pos, _, _ = packet.read4ByteTxRx(port, id, 132)
            if pos > 0x7FFFFFFF:
                pos -= 0x100000000

            angle = ticks_to_deg(pos)
            error = CENTERS_DEG[id] - angle
            pwm = int(K * error)
            pwm = max(-885, min(885, pwm))

            print(f"ID {id} | Angle: {angle:7.2f}°  |  Error: {error:7.2f}°  |  PWM: {pwm:6d}")
            packet.write2ByteTxRx(port, id, 100, pwm & 0xFFFF)

        print("---")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    for id in IDS:
        packet.write2ByteTxRx(port, id, 100, 0)
        packet.write1ByteTxRx(port, id, 64, 0)
    port.closePort()