class Obstacle:
    """
    Abstract class containing obstacles position and shape information.
    """
    def __init__(self):
        pass

    def contains(self, point, margin):
        """
        Checks if the obstacle contains a point, with a given security margin.
        :param point: Point to be checked.
        :param margin: A margin for the drone to be in this point and not collide.
        :return: True if the obstacle contains point.
        """
        pass
