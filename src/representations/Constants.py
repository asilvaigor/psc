from std_msgs.msg import ColorRGBA

# Rate to run the main loop to control the drones
RATE = 30  # Hz

# Room limits for the drones to be in.
# TODO: put good values
MIN_X = -0.5  # m
MAX_X = 0.5  # m
MIN_Y = -0.5  # m
MAX_Y = 0.5  # m
MIN_Z = 0  # m
MAX_Z = 1.5  # m

# Maximum velocities
# TODO: put good values
MAX_VEL_X = 0.375  # m/s
MAX_VEL_Y = 0.375  # m/s
MAX_VEL_Z = 0.375  # m/s
MAX_VEL_YAW = 0.5  # rad/s

# Maximum acceleration
# TODO: put good values
MAX_ACC_X = 0.03  # m/s^2
MAX_ACC_Y = 0.03  # m/s^2
MAX_ACC_Z = 0.03  # m/s^2
MAX_ACC_YAW = 0.03  # rad/s^2

# TODO: define maximum path curvature

# Margin for a drone to pass next to an obstacle.
OBSTACLE_MARGIN = 0.2  # m

# Colors
BLUE = ColorRGBA(0.0, 0.0, 1.0, 1.0)
RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.0)
PINK = ColorRGBA(1.0, 0.0781, 0.5742, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
ORANGE = ColorRGBA(0.9726, 0.5820, 0.0273, 1.0)
COLORS = [BLUE, YELLOW, PINK, GREEN, WHITE, ORANGE]
