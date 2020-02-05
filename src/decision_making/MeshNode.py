import math


class MeshNode:
    """
    Node for the graph representing the world mesh, a discrete representation of the space.
    """

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.__x = x
        self.__y = y
        self.__z = z
        self.edges = []

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    def add_edge(self, node):
        """
        Adds an edge to the node.
        :param node:
        """
        self.edges.append(node)
        node.edges.append(self)

    def dist(self, node):
        """
        Returns distance between two nodes.
        @param node:
        """
        return math.hypot(math.hypot(self.x - node.x, self.y - node.y), self.z - node.z)

    def __eq__(self, node):
        """
        Checks if two nodes are equal.
        :param node:
        """
        return self.dist(node) < 1e-3

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
