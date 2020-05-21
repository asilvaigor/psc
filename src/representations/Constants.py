from std_msgs.msg import ColorRGBA

# Rate to update the drones in rviz
VISUALIZATION_RATE = 30  # Hz

# Number of drones
N_DRONES = 12

# Double precision
GROUND_EPS = 0.05  # m
EPS = 1e-6  # m

# Room limits for the drones to be in.
# TODO: put good values
MIN_X = -1.1  # m
MAX_X = 1.1  # m
MIN_Y = -1.5  # m
MAX_Y = 1.5  # m
MIN_Z = 0  # m
MAX_Z = 1.5  # m

# Maximum velocities
# TODO: put good values
MAX_VEL = 0.25  # m/s
MAX_VEL_YAW = 0.5  # rad/s

# TODO: define maximum path curvature

# Margin for a drone to pass next to an obstacle.
OBSTACLE_MARGIN = 0.2  # m

# Maximum distance between two edge vertex.
# Not used for mesh generation!!
MESH_EDGE_DIST = 0.5  # m

# Height of the drone when it is on the floor
DRONE_HEIGHT = 0.035

# Colors for drones
BLUE = ColorRGBA(0.0, 0.0, 1.0, 1.0)
RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
PINK = ColorRGBA(1.0, 0.0781, 0.5742, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
ORANGE = ColorRGBA(0.9726, 0.5820, 0.0273, 1.0)
COLORS = [BLUE, YELLOW, PINK, GREEN, WHITE, ORANGE]

# Colors for trajectory
BLUE_T = ColorRGBA(0.0, 0.0, 1.0, 0.2)
RED_T = ColorRGBA(1.0, 0.0, 0.0, 0.2)
YELLOW_T = ColorRGBA(1.0, 1.0, 0.0, 0.2)
GREEN_T = ColorRGBA(0.0, 1.0, 0.0, 0.2)
PINK_T = ColorRGBA(1.0, 0.0781, 0.5742, 0.2)
WHITE_T = ColorRGBA(1.0, 1.0, 1.0, 0.2)
ORANGE_T = ColorRGBA(0.9726, 0.5820, 0.0273, 0.2)
COLORS_T = [BLUE_T, YELLOW_T, PINK_T, GREEN_T, WHITE_T, ORANGE_T]

# Marker size
MARKER_SIZE_X = 0.2
MARKER_SIZE_Y = 0.05
MARKER_SIZE_Z = 0.05
MARKER_ARROW_SIZES = [MARKER_SIZE_X, MARKER_SIZE_Y, MARKER_SIZE_Z]
MARKER_LINE_SIZE_X = 0.05
