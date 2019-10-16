from perception.vision.Vision import Vision
from representations.obstacles.ObstacleCollection import ObstacleCollection


class Perception:
    """
    Contains obstacle detection including vision and tracking, and eventually position filters for the drones.
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
        return ObstacleCollection()
