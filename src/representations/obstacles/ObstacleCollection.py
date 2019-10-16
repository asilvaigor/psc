class ObstacleCollection:
    """
    Collection of obstacle objects in the world.
    """

    def __init__(self):
        """
        Basic constructor. Stores a list with all the obstacles.
        """
        self.__obstacles = []

    def insert(self, obstacle):
        """
        Inserts a new obstacle into the list of obstacles.
        :param obstacle: Obstacle to be inserted.
        """
        self.__obstacles += obstacle

    def collides(self, trajectory, margin):
        """
        Checks if the drone will collide with any of the obstacles by following the given trajectory.
        :param trajectory: Trajectory the drone should follow.
        :param margin: A margin for the drone to pass by an obstacle and not collide.
        :return: True if the trajectory collides with any obstacle.
        """
        for obstacle in self.__obstacles:
            if obstacle.collides(trajectory, margin):
                return True
        return False

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
