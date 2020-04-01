class Node:
    """
    Abstract Node class to use in graph algorithms.
    """

    def __init__(self):
        self.edges = []

    def add_edge(self, node):
        """
        Adds an edge to the node.
        :param node:
        """
        if node in self.edges:
            return

        self.edges.append(node)
        node.edges.append(self)

    def dist(self, arg):
        """
        Returns the distance to a certain object.
        :param arg: Object to calculate distance. Can be Point, Pose, StablePose or MeshNode.
        :return: float, distance to the object.
        """
        raise NotImplementedError()

    def __eq__(self, node):
        """
        Checks if two nodes are equal.
        :param node:
        """
        raise NotImplementedError()

    def __repr__(self):
        """
        Used for printing node.
        :return: String representing MeshNode.
        """
        raise NotImplementedError()

    def __hash__(self):
        """
        Hash function to use in dicts.
        :return: HashCode associated to Node.
        """
        raise NotImplementedError()
