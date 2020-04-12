import math

from decision_making.Node import Node
from representations.Constants import EPS
from representations.StablePose import StablePose
from representations.Point import Point


class DeltaNode(Node):
    """
    Node for the graph in the Delta space, which is a 2D space where axis 1 is the first path and
    axis 2 is the second path. Units are in meters, representing the distance from the start of the
    path.
    """

    def __init__(self, x, y):
        Node.__init__(self)
        self.__pos = Point(x, y)

    @property
    def x(self):
        return self.__pos.x

    @property
    def y(self):
        return self.__pos.y

    def position(self):
        return self.__pos

    def dist(self, arg):
        """
        Returns the distance to a certain object.
        :param arg: Object to calculate distance. Can be Point, Pose, StablePose or MeshNode.
        :return: float, distance to the object.
        """
        if isinstance(arg, DeltaNode):
            return self.__pos.dist(arg.position())
        elif isinstance(arg, Point):
            return self.__pos.dist(arg)
        else:
            raise ValueError("DeltaNode can't calculate distances to object of type " +
                             type(arg).__name__)

    def __eq__(self, node):
        """
        Checks if two nodes are equal.
        :param node:
        """
        return self.dist(node) < EPS

    def __hash__(self):
        """
        Hash function to use in dicts.
        :return: HashCode associated to Node.
        """
        return self.x.__hash__() ^ self.y.__hash__()

    def __repr__(self):
        """
        Used for printing node.
        :return: String representing MeshNode.
        """
        return "[" + str(self.x) + ", " + str(self.y) + "]"
