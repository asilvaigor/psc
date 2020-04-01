from geometry_msgs.msg import Pose, Point
import math

from decision_making.Node import Node
from representations.Constants import GROUND_PRECISION, PRECISION
from representations.StablePose import StablePose


class MeshNode(Node):
    """
    Node for the graph representing the world mesh, a discrete representation of the space.
    """

    def __init__(self, x=0.0, y=0.0, z=0.0):
        Node.__init__(self)
        self.__x = x
        self.__y = y
        self.__z = z

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    def position(self):
        return Point(self.x, self.y, self.z)

    def dist(self, arg):
        """
        Returns the distance to a certain object.
        :param arg: Object to calculate distance. Can be Point, Pose, StablePose or MeshNode.
        :return: float, distance to the object.
        """
        if isinstance(arg, MeshNode):
            return math.hypot(math.hypot(self.x - arg.x, self.y - arg.y), self.z - arg.z)
        elif isinstance(arg, Point):
            return math.hypot(math.hypot(self.x - arg.x, self.y - arg.y), self.z - arg.z)
        elif isinstance(arg, Pose):
            return self.dist(arg.position)
        elif isinstance(arg, StablePose):
            return self.dist(arg.position)
        else:
            raise ValueError("MeshNode can't calculate distances to object of type " +
                             type(arg).__name__)

    def is_on_ground(self):
        """
        Checks if node is on the ground.
        :return: True or False
        """
        return self.z < GROUND_PRECISION

    def __eq__(self, node):
        """
        Checks if two nodes are equal.
        :param node:
        """
        return self.dist(node) < PRECISION

    def __repr__(self):
        """
        Used for printing node.
        :return: String representing MeshNode.
        """
        return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + "]"

    def __hash__(self):
        """
        Hash function to use in dicts.
        :return: HashCode associated to Node.
        """
        return self.x.__hash__() ^ self.y.__hash__() ^ self.z.__hash__()

    def __lt__(self, node):
        """
        Overloads operator less than "<". Compares x, y and z values.
        :param node:
        :return: Bool True if self should be placed before node.
        """
        if abs(self.x - node.x) < PRECISION:
            if abs(self.y - node.y) < PRECISION:
                return self.z < node.z
            else:
                return self.y < node.y
        else:
            return self.x < node.x
