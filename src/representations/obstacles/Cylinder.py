import copy
from geometry_msgs.msg import Point
from math import hypot

from representations.obstacles.Obstacle import Obstacle
from representations.Constants import OBSTACLE_MARGIN


class Cylinder(Obstacle):
    """
    Stores a cylinder object for obstacles avoidance. This cylinder is infinite in one axis.
    """

    def __init__(self, p, radius, axis):
        """
        Basic constructor.
        :param p: Point on center of the circle of the cylinder.
        :param radius: Radius of the center of the cylinder.
        :param axis: Char 'x', 'y' or 'z' which represents axis in which the cylinder is extruded.
        """
        Obstacle.__init__(self)
        self.__p = p
        self.__radius = radius
        self.__axis = axis

    @property
    def position(self):
        return self.__p

    @property
    def radius(self):
        return self.__radius

    @property
    def axis(self):
        return self.__axis

    def contains(self, point, margin=OBSTACLE_MARGIN):
        """
        Checks if the cylinder contains a point, with a given security margin.
        :param point: Point to be checked.
        :param margin: A margin for the drone to be in this point and not collide.
        :return: True if the cylinder contains point.
        """
        p1 = copy.copy(self.__p)
        p2 = copy.copy(point)
        if self.__axis == 'x':
            p1.x = 0
            p2.x = 0
        elif self.__axis == 'y':
            p1.y = 0
            p2.y = 0
        else:
            p1.z = 0
            p2.z = 0

        dist = hypot(p1.x-p2.x, hypot(p1.y-p2.y, p1.z-p2.z))
        return dist < self.__radius + margin
