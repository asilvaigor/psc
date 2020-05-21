import sys
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(1, dir_path + '/../../../generate_mesh/build/')
import mesh

from MeshNode import MeshNode
from representations.obstacles.Cylinder import Cylinder
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MAX_Z
from representations.Constants import MESH_EDGE_DIST
from representations.Constants import DRONE_HEIGHT
from representations.Segment import Segment
from representations.Point import Point


class CGALMesh:
    """
    Represents the world mesh uniformly - nodes separated by a constant distance.
    """

    def __init__(self, angle=3.20447570e+01, size=2.15358173e-01, approximation=1.86773522e-02
                 , radiusedge=1.90374963e+00, ratio=3.05491966e-01):
        """
        Default constructor.
        """
        self.__nodes = []
        self.__angle = angle
        self.__size = size
        self.__approximation = approximation
        self.__radiusedge = radiusedge
        self.__ratio = ratio

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
        cyl_c1_list = []
        cyl_c2_list = []
        cyl_r_list = []
        cyl_dir_list = []
        for obs in obstacle_collection.obstacles:
            if isinstance(obs, Cylinder):
                if obs.axis == 'x':
                    cyl_dir_list.append(0)
                    cyl_c1_list.append(obs.position.y + abs(MIN_Y))
                    cyl_c2_list.append(obs.position.z)
                elif obs.axis == 'y':
                    cyl_dir_list.append(1)
                    cyl_c1_list.append(obs.position.x + abs(MIN_X))
                    cyl_c2_list.append(obs.position.z)
                elif obs.axis == 'z':
                    cyl_dir_list.append(2)
                    cyl_c1_list.append(obs.position.x + abs(MIN_X))
                    cyl_c2_list.append(obs.position.y + abs(MIN_Y))

                cyl_r_list.append(obs.radius)

        x_dim = MAX_X - MIN_X
        y_dim = MAX_Y - MIN_Y
        z_dim = MAX_Z
        mesh_nodes = mesh.generateMesh([x_dim, y_dim, z_dim],
                                       cyl_c1_list, cyl_c2_list, cyl_r_list, cyl_dir_list,
                                       self.__angle, self.__size, self.__approximation,
                                       self.__radiusedge, self.__ratio)
        for _ in mesh_nodes:
            self.__nodes.append(MeshNode())
        for n in mesh_nodes:
            self.__nodes[n.index].x = n.x - abs(MIN_X)
            self.__nodes[n.index].y = n.y - abs(MIN_Y)
            self.__nodes[n.index].z = n.z
            for edge in n.adj:
                self.__nodes[n.index].add_edge(self.__nodes[edge])

        def add_node(n):
            for node in self.__nodes:
                # Only short edges that are not on the ground or through obstacles
                if n.dist(node) < MESH_EDGE_DIST and (n.z > DRONE_HEIGHT or node.z > DRONE_HEIGHT):
                    valid_edge = True
                    for obs in obstacle_collection.obstacles:
                        if isinstance(obs, Cylinder):
                            cylinder_seg = Segment(Point(obs.position), Point(obs.position))
                            if obs.axis == 'x':
                                cylinder_seg.b += Point(1000, 0, 0)
                            elif obs.axis == 'y':
                                cylinder_seg.b += Point(0, 1000, 0)
                            else:
                                cylinder_seg.b += Point(0, 0, 1000)
                            edge_seg = Segment(Point(n.position()), Point(node.position()))

                            if cylinder_seg.min_distance(edge_seg) < obs.radius:
                                valid_edge = False
                    if valid_edge:
                        n.add_edge(node)

        # Adding drone nodes
        drone_nodes = {}
        if drone_poses is not None:
            for drone_id in drone_poses:
                p = drone_poses[drone_id].position
                drone_nodes[drone_id] = MeshNode(p.x, p.y, p.z)
                add_node(drone_nodes[drone_id])

        # Adding goal nodes
        goal_nodes = {}
        if goal_poses is not None:
            for drone_id in goal_poses:
                p = goal_poses[drone_id]
                goal_nodes[drone_id] = MeshNode(p.x, p.y, p.z)
                add_node(goal_nodes[drone_id])

        return drone_nodes, goal_nodes
