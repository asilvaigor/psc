from geometry_msgs.msg import Point

from perception.vision.Vision import Vision
from representations.obstacles.Cylinder import Cylinder
from representations.obstacles.ObstacleCollection import ObstacleCollection


class Perception:
    """
    Contains obstacle detection including vision and tracking, and eventually position filters for
    the drones.
    """

    def __init__(self):
        """
        Basic constructor.
        """
        self.__vision = Vision()

    def perceive(self):
        """
        Detects obstacles on the scene and updates their trackers.
        :return: obstacle_collection
        """
        # FIXME: Putting virtual objects to test algorithms
        c1 = Cylinder(Point(0.5, 0.5, 0), 0.2, 'z')
        c2 = Cylinder(Point(-0.5, -0.3, 0), 0.2, 'z')
        c3 = Cylinder(Point(0, 0.3, 1), 0.2, 'x')
        return ObstacleCollection([c1, c2, c3])
