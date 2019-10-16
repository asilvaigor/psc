class Obstacle:
    """
    Abstract class containing obstacles position and shape information.
    """
    def __init__(self):
        pass

    def collides(self, trajectory, margin):
        """
        Checks if the drone will collide with the obstacle by following the given trajectory.
        :param trajectory: Trajectory the drone should follow.
        :param margin: A margin for the drone to pass by the obstacle and not collide.
        :return: True if the trajectory collides with the obstacle.
        """
        pass

    def contains(self, point, margin):
        """
        Checks if the obstacle contains a point, with a given security margin.
        :param point: Point to be checked.
        :param margin: A margin for the drone to be in this point and not collide.
        :return: True if the obstacle contains point.
        """
        pass
