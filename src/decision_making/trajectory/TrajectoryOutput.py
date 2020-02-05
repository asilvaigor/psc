# Probably what the drones uses for
class TrajectoryOutput:
    def __init__(self):
        self.pos = None  # position [m]
        self.vel = None  # velocity [m/s]
        self.acc = None  # acceleration [m/s^2]
        self.omega = None  # angular velocity [rad/s]
        self.yaw = None  # yaw angle [rad]
