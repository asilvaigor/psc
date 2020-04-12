import numpy as np
from geometry_msgs.msg import Point

from decision_making.MeshNode import MeshNode
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z, EPS


class UniformMesh:
    """
    Represents the world mesh uniformly - nodes separated by a constant distance.
    """

    def __init__(self, delta):
        """
        Default constructor.
        :param delta: Minimum separation between two nodes.
        """
        self.__nodes = []
        self.__delta = delta

    @property
    def nodes(self):
        return self.__nodes

    def discretize(self, obstacle_collection, drone_poses, goal_poses):
        """
        Creates a discrete version of the world. It sets the mesh_node parameter in each drone, and
        also its two variables, __nodes and __goal_nodes.
        :param obstacle_collection: ObstacleCollection object with obstacles.
        :param drone_poses: Dict of StablePose objects, for the poses for each drone_id.
        :param goal_poses: Dict of Pose objects for each drone's final pose.
        :return drone_nodes: Dict of MeshNode objects for the node for each drone.
        :return goal_nodes: Dict of MeshNode objects for nodes for each goal.
        """
        nodes_dict = {}
        d = self.__delta

        # Creates every node
        for x in np.arange(MIN_X, MAX_X, self.__delta):
            for y in np.arange(MIN_Y, MAX_Y, self.__delta):
                for z in np.arange(MIN_Z, MAX_Z, self.__delta):
                    if not obstacle_collection.contains(Point(x, y, z)):
                        cur = MeshNode(x, y, z)
                        nodes_dict[round(x, 3), round(y, 3), round(z, 3)] = cur

        # Adds edges between nodes.
        def add_edge(k1, k2):
            if k2 in nodes_dict and not \
                    (nodes_dict[k1].is_on_ground() and nodes_dict[k2].is_on_ground()):
                nodes_dict[k1].add_edge(nodes_dict[k2])

        for k in nodes_dict:
            add_edge(k, (round(k[0] + d, 3), k[1], k[2]))
            add_edge(k, (round(k[0] - d, 3), k[1], k[2]))
            add_edge(k, (k[0], round(k[1] + d, 3), k[2]))
            add_edge(k, (k[0], round(k[1] - d, 3), k[2]))
            add_edge(k, (k[0], k[1], round(k[2] + d, 3)))
            add_edge(k, (k[0], k[1], round(k[2] - d, 3)))

        def add_node(x, y, z, n):
            k1 = (round(x, 3), round(y, 3), round(z, 3))
            nodes_dict[k1] = n
            for k2 in nodes_dict:
                if k1 != k2 and not \
                        (nodes_dict[k1].is_on_ground() and nodes_dict[k2].is_on_ground()) and \
                        nodes_dict[k1].dist(nodes_dict[k2]) < 1.7 * self.__delta + EPS:
                    add_edge(k1, k2)

        # Adding drone nodes
        drone_nodes = {}
        if drone_poses is not None:
            for drone_id in drone_poses:
                p = drone_poses[drone_id].position
                drone_nodes[drone_id] = MeshNode(p.x, p.y, p.z)
                add_node(p.x, p.y, p.z, drone_nodes[drone_id])

        # Adding goal nodes
        goal_nodes = {}
        if goal_poses is not None:
            for drone_id in goal_poses:
                p = goal_poses[drone_id]
                goal_nodes[drone_id] = MeshNode(p.x, p.y, p.z)
                add_node(p.x, p.y, p.z, goal_nodes[drone_id])

        # Converts dict to list
        for node in nodes_dict.values():
            self.__nodes.append(node)

        return drone_nodes, goal_nodes
