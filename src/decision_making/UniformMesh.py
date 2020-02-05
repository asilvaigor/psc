import numpy as np
from geometry_msgs.msg import Point

from decision_making.MeshNode import MeshNode
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z, PRECISION


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
        self.__goal_nodes = {}
        self.__delta = delta

    @property
    def nodes(self):
        return self.__nodes

    @property
    def goal_nodes(self):
        return self.__goal_nodes

    def discretize(self, drones, obstacle_collection, goal_poses):
        """
        Creates a discrete version of the world. It sets the mesh_node parameter in each drone, and
        also its two variables, __nodes and __goal_nodes.
        :param drones: Dict of Crazyflie representing the drones.
        :param obstacle_collection: ObstacleCollection object with obstacles.
        :param goal_poses: Dict of Pose objects for each drone's final pose.
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
            if k2 in nodes_dict:
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
                if k1 != k2 and nodes_dict[k1].dist(nodes_dict[k2]) < self.__delta + PRECISION:
                    add_edge(k1, k2)

        # Adding drone nodes
        for drone_id in drones:
            p = drones[drone_id].pose.position
            drones[drone_id].set_mesh_node(MeshNode(p.x, p.y, p.z))
            add_node(p.x, p.y, p.z, drones[drone_id].mesh_node)

        # Adding goal nodes
        for drone_id in goal_poses:
            p = goal_poses[drone_id]
            self.__goal_nodes[drone_id] = MeshNode(p.x, p.y, p.z)
            add_node(p.x, p.y, p.z, self.__goal_nodes[drone_id])

        # Converts dict to list
        for node in nodes_dict.values():
            self.__nodes.append(node)
