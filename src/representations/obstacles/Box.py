from representations.obstacles.Obstacle import Obstacle


class Box(Obstacle):
    """
    Stores a box object for obstacles avoidance. A box consists of a 3-dimensional rectangle.
    """

    def __init__(self, p1, p2):
        """
        Basic constructor.
        :param p1: Point with the minimum x, y and z coordinates from the 8 corner points.
        :param p2: Point with the maximum x, y and z coordinates from the 8 corner points.
        """
        Obstacle.__init__(self)
        self.__p1 = p1
        self.__p2 = p2

    def contains(self, point, margin):
        """
        Checks if the box contains a point, with a given security margin.
        :param point: Point to be checked.
        :param margin: A margin for the drone to be in this point and not collide.
        :return: True if the box contains point.
        """
        pass
