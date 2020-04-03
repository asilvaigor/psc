import math
from geometry_msgs import msg

from representations.Constants import EPS


class Point:
    def __init__(self, *args):
        """
        Default constructor. Receives geometry_msgs.Point or (x, y, z).
        :param args: See below.

        :Arguments:
            * point: geometry_msgs.Point. Can also be passed as (x, y, z).
            * x: x for the Point.
            * y: y for the Point.
            * z: z for the Point.
        """
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        if len(args) == 1 and isinstance(args[0], msg.Point):
            self.x = args[0].x
            self.y = args[0].y
            self.z = args[0].z
        else:
            if len(args) > 0:
                self.x = args[0]
            if len(args) > 1:
                self.y = args[1]
            if len(args) > 2:
                self.z = args[2]

    def dist(self, p):
        """
        Distance to another point.
        :param p: Another point.
        :return: float value, distance between two points.
        """
        return math.hypot(math.hypot(self.x - p.x, self.y - p.y), self.z - p.z)

    def inner(self, p):
        """
        Inner product with another point.
        :param p: Another point.
        :return: float value, inner product between two points.
        """
        return self.x * p.x + self.y * p.y + self.z * p.z

    def cross(self, p):
        """
        Cross product with another point.
        :param p: Another point.
        :return: float value, cross product between two points.
        """
        return Point(self.y * p.z - self.z * p.y,
                     self.z * p.x - self.x * p.z,
                     self.x * p.y - self.y * p.x)

    def norm(self):
        """
        Module operator.
        :return: float value, always positive.
        """
        return math.sqrt(self.inner(self))

    def dist_to_segment(self, segment):
        """
        Minimum distance to a segment.
        :param segment: Segment to be calculated.
        :return: float value, positive, distance to the given segment.
        """
        return segment.dist_to_point(self)

    def __eq__(self, p):
        """
        Equal operator.
        :param p:
        :return: Boolean
        """
        return self.dist(p) < EPS

    def __neg__(self):
        """
        Operator -Point.
        :return: Negative of the current point.
        """
        return Point(-self.x, -self.y, -self.z)

    def __add__(self, p):
        """
        Operator Point + Point.
        :param p: Point to be added.
        :return: Sum of both points.
        """
        return Point(self.x + p.x, self.y + p.y, self.z + p.z)

    def __sub__(self, p):
        """
        Operator Point - Point.
        :param p: Point to be subtracted.
        :return: Subtraction of points.
        """
        return self + p.__neg__()

    def __mul__(self, k):
        """
        Operator Point * scalar
        :param k: float, scalar value.
        :return: Point multiplied
        """
        return Point(self.x * k, self.y * k, self.z * k)

    def __div__(self, k):
        """
        Operator Point / scalar
        :param k: float, scalar value.
        :return: Point divided
        """
        return Point(self.x / k, self.y / k, self.z / k)

    def __repr__(self):
        """
        Used for printing.
        :return: String representing Point.
        """
        return "[" + str(self.x) + ", " + str(self.y) + ", " + \
               str(self.z) + "]"
