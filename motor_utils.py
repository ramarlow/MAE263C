from dynio import dxl   # I like https://pypi.org/project/dynamixel-controller/ because the function calls are easier to remember
                        # if we really need high performance i think it's fastest to use dynamixel sdk with bulk read and write options
import numpy as np

def set_PID_gains(motor:dxl.DynamixelMotor, kP=256, kI=0, kD=0):
    '''
    write PID gains to a servo. if no gains are passed will set to default values
    '''
    if kP != None:
        kP_scaled = kP / 8
        if kP_scaled > 254 or kP_scaled < 0: raise ArithmeticError('kP outside of acceptable bounds (0-2032)')
        motor.write_control_table('P_Gain',int(kP_scaled))
    if kI != None:
        kI_scaled = kI * 1000/2048
        if kI_scaled > 254 or kI_scaled < 0: raise ArithmeticError('kI outside of acceptable bounds (0-520)')
        motor.write_control_table('I_Gain',int(kI_scaled))
    if kD != None:
        kD_scaled = kD * 4/1000
        if kD_scaled > 254 or kD_scaled < 0: raise ArithmeticError('kD outside of acceptable bounds (0-63500)')
        motor.write_control_table('D_Gain',int(kD_scaled))

class MotorArr:
    def __init__(self, port:str, baud_rate:int, device_ids:list):
        self.dxl_comm = dxl.DynamixelIO(port, baud_rate) # setup a single communication object to handle serial data
        self.motor_arr = np.array([self.dxl_comm.new_mx28(dev_id, 2) for dev_id in device_ids])

    def set_positions(self, positions): # position setpoints
        for i, motor in enumerate(self.motor_arr):
            if positions[i] == None: continue
            motor.set_position(int(positions[i])) # int casting because otherwise the bitwise math gets angry

    def set_angles(self, angles): # angle setpoints
        for i, motor in enumerate(self.motor_arr):
            if angles[i] == None: continue
            motor.set_angle(angles[i])

    def set_velocities(self, vels=None): # velocity setpoints
        if vels == None: # if nothing is passed default to stopping everything
            for motor in self.motor_arr:
                motor.set_velocity(0)
        else:
            for i, motor in enumerate(self.motor_arr):
                if vels[i] == None: continue
                if vels[i] < 0:
                    motor.set_velocity(int(vels[i]))
                else:
                    motor.set_velocity(int(vels[i]))

    def set_mode(self, mode, *args):
        # i should add support for setting limits but for now im not gonna bother
        mode = mode.lower()
        mode_table = {'position':3, 'pos':3, # a few aliases that i can think of
                      'velocity':1, 'vel':1,
                      'extended position':4, 'ext pos':4, 'ext_pos':4,
                      'pwm':16, 'voltage':16}
        for motor in self.motor_arr:
            try:
                motor.write_control_table("Operating_Mode", mode_table[mode])
            except:
                pass

    def set_gains(self, gains=None):
        if gains == None: # if a single None val is passed, reset all gains
            for motor in self.motor_arr:
                set_PID_gains(motor)
        else:
            for i, motor in enumerate(self.motor_arr):
                set_PID_gains(motor, *gains[i,:])

    def set_torques(self, torques=None):
        if torques == None: # if nothing is passed default to disabling everything
            for motor in self.motor_arr:
                motor.torque_disable()
        else:
            for i, motor in enumerate(self.motor_arr):
                if torques[i]==True:
                    motor.torque_enable()
                elif torques[i]==False:
                    motor.torque_disable()

    def torque_enable(self): # need to call this before anything will move
        for motor in self.motor_arr:
            motor.torque_enable()

    def torque_disable(self):
        for motor in self.motor_arr:
            motor.torque_disable()

    def get_loads(self): # returns inferred the loads applied to motors
        loads = np.zeros_like(self.motor_arr)
        for i, motor in enumerate(self.motor_arr):
            load = motor.get_current()
            if load > 1023:
                load = load - 65535
            loads[i] = load/1000
        return loads

    def get_positions(self):
        return np.array([motor.get_position() for motor in self.motor_arr])
    
    def get_angles(self):
        return np.array([motor.get_angle() for motor in self.motor_arr])
    
    def apply_write_table(self, data_name, vals): # more general function to write to any value in the control table
        for i, motor in enumerate(self.motor_arr):
            if vals[i] == None: continue
            motor.write_control_table(data_name, int(vals[i]))

    def apply_read_table(self, data_name):
        return np.array([motor.read_control_table(data_name) for motor in self.motor_arr])