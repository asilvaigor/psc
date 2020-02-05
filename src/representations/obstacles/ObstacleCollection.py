class ObstacleCollection:
    """
    Collection of obstacle objects in the world.
    """

    def __init__(self, obstacles=None):
        """
        Basic constructor. Stores a list with all the obstacles.
        :param obstacles: List of Obstacle objects.
        """
        if obstacles is None:
            obstacles = []
        self.__obstacles = obstacles

    @property
    def obstacles(self):
        """
        Getter for obstacles.
        :return: List of Obstacle objects.
        """
        return self.__obstacles

    def insert(self, obstacle):
        """
        Inserts a new obstacle into the list of obstacles.
        :param obstacle: Obstacle to be inserted.
        """
        self.__obstacles.append(obstacle)

    def contains(self, point, margin):
        """
        Checks if any of the obstacles contains a point, with a given security margin.
        :param point: Point to be checked.
        :param margin: A margin for the drone to be in this point and not collide with any obstacle.
        :return: True if any obstacle contains point.
        """
        for obstacle in self.__obstacles:
            if obstacle.contains(point, margin):
                return True
        return False
