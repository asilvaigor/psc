import math


class MeshNode:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.__x = x
        self.__y = y
        self.__z = z
        self.children = []

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    def add_child(self, node):
        self.children.append(node)

    def dist(self, node):
        """
        Returns distance between two nodes.
        @param node:
        """
        return math.hypot(math.hypot(self.x - node.x, self.y - node.y), self.z - node.z)
