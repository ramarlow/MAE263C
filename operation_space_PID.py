import numpy as np

class PDController:
    def __init__(self, setpoint, kp=50.0, kd=10.0, dt=0.01):
        self.setpoint = np.array(setpoint, dtype=float)
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.velocity = np.zeros(2)
        self.prev_pos = None

    def update(self, x, y):
        """
        Takes current (x, y) position in metres.
        Returns force vector [Fx, Fy] in Newtons.
        """
        current_pos = np.array([x, y], dtype=float)

        # Estimate velocity from position difference if not provided externally
        if self.prev_pos is not None:
            self.velocity = (current_pos - self.prev_pos) / self.dt
        self.prev_pos = current_pos

        error = self.setpoint - current_pos
        force = self.kp * error - self.kd * self.velocity
        return force