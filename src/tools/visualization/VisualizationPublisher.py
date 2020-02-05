import math
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

from representations.Constants import COLORS
from representations.Constants import COLORS_T
from representations.Constants import MARKER_ARROW_SIZES
from representations.Constants import MARKER_LINE_SIZE_X
from representations.Constants import WHITE, RED
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z
from representations.obstacles.Cylinder import Cylinder


class VisualizationPublisher:

    def __init__(self, drones):
        """
        Basic constructor
        """
        self.__drones = drones
        self.__past_path_markers = MarkerArray()
        self.__drone_color_map = {}
        for drone_id in drones:
            self.add_in_trajectory(drones, drones[drone_id])

        self.__drone_publisher = rospy.Publisher('visualization_drones', MarkerArray, queue_size=10)
        self.__past_path_publisher = rospy.Publisher('visualization_ppath', MarkerArray,
                                                     queue_size=10)
        self.__future_path_publisher = rospy.Publisher('visualization_fpath', Marker, queue_size=10)
        self.__goal_publisher = rospy.Publisher('visualization_goals', MarkerArray, queue_size=10)
        self.__obstacle_publisher = rospy.Publisher('visualization_obstacles', MarkerArray,
                                                    queue_size=10)
        self.__mesh_publisher = rospy.Publisher('visualization_mesh', Marker, queue_size=10)

    def visualize(self, goal_poses=None):
        """
        Updates all markers on rviz.
        :param goal_poses: Dict of goal Pose for each drone.
        """
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones.values():
            drones_markers.markers.append(self.__get_pose_marker(
                drone.pose, COLORS[self.__drone_color_map[drone.id]], drone.id, 1))

        # Here we update the past path
        self.__update_past_path()

        # Here we update the goal poses
        if goal_poses is not None:
            self.__update_goal_poses(goal_poses)

        # Publish the array of markers and the trajectory
        self.__drone_publisher.publish(drones_markers)
        self.__past_path_publisher.publish(self.__past_path_markers)

    def visualize_world(self, mesh=None, obstacle_collection=None):
        """
        Updates mesh and obstacles markers in rviz.
        :param mesh: Mesh object which describes the space.
        :param obstacle_collection: ObstacleCollection with obstacles in the scene
        """
        # Here we update the obstacles
        if obstacle_collection is not None:
            self.__update_obstacles(obstacle_collection)

        # Here we update the mesh
        if mesh is not None:
            self.__update_mesh(mesh)

    def visualize_paths(self):
        for drone_id in self.__drones:
            if len(self.__drones[drone_id].path) > 0:
                self.__update_future_path(drone_id, self.__drones[drone_id].path)

    def __update_past_path(self):
        # For each drone we add his position in the moment
        for m in self.__past_path_markers.markers:
            m.points.append(self.new_point(self.__drones[m.id]))

    def __update_goal_poses(self, goal_poses):
        """
        Refreshes goal poses in rviz.
        :param goal_poses: Dict of StablePoses for each drone.
        """
        markers = MarkerArray()
        if 0 in goal_poses and len(self.__drones) % 2 == 0:
            markers.markers.append(self.__get_pose_marker(
                goal_poses[0].to_ros(), WHITE, 0, 0.7))
        for drone_id in self.__drones:
            if drone_id in goal_poses:
                markers.markers.append(self.__get_pose_marker(
                    goal_poses[drone_id].to_ros(),
                    COLORS[self.__drone_color_map[drone_id]], drone_id, 0.5))

        self.__goal_publisher.publish(markers)

    def __update_obstacles(self, obstacle_collection):
        """
        Refreshes obstacles in rviz. Currently supports only cylinders.
        :param obstacle_collection: ObstacleCollection with obstacles in the scene.
        """
        markers = MarkerArray()
        for obstacle in obstacle_collection.obstacles:
            if isinstance(obstacle, Cylinder):
                m = Marker()
                m.header.frame_id = str(1)
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.id = obstacle_collection.obstacles.index(obstacle)
                m.color = WHITE

                m.scale.x = 2 * obstacle.radius
                m.scale.y = 2 * obstacle.radius

                m.pose.position = obstacle.position
                orientation = []
                if obstacle.axis == 'x':
                    orientation = quaternion_from_euler(0, math.pi / 2, 0)
                    m.pose.position.x = 0
                    m.scale.z = MAX_X - MIN_X
                elif obstacle.axis == 'y':
                    orientation = quaternion_from_euler(math.pi / 2, 0, 0)
                    m.pose.position.y = 0
                    m.scale.z = MAX_Y - MIN_Y
                elif obstacle.axis == 'z':
                    orientation = quaternion_from_euler(0, 0, 0)
                    m.pose.position.z = (MAX_Z - MIN_Z) / 2
                    m.scale.z = MAX_Z - MIN_Z
                m.pose.orientation.x = orientation[0]
                m.pose.orientation.y = orientation[1]
                m.pose.orientation.z = orientation[2]
                m.pose.orientation.w = orientation[3]

                markers.markers.append(m)

        self.__obstacle_publisher.publish(markers)

    def __update_mesh(self, mesh):
        """
        Refreshes mesh in rviz.
        :param mesh: Mesh object representing the space.
        """
        m = Marker()
        m.header.frame_id = str(1)
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.id = 1
        m.color = RED

        m.scale.x = 0.001

        edges_visited = set()
        for n1 in mesh.nodes:
            for n2 in n1.edges:
                if n1 < n2 and (n1, n2) not in edges_visited:
                    m.points.append(n1.position())
                    m.points.append(n2.position())
                    edges_visited.add((n1, n2))
                elif n2 < n1 and (n2, n1) not in edges_visited:
                    m.points.append(n2.position())
                    m.points.append(n1.position())
                    edges_visited.add((n2, n1))

        self.__mesh_publisher.publish(m)

    def __update_future_path(self, drone_id, path):
        """
        Refreshes future path in rviz.
        :param drone_id: id of the drone in the path.
        :param path: List of MeshNode with path to follow.
        """
        m = Marker()
        m.header.frame_id = str(1)
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.id = drone_id
        m.color = COLORS[self.__drone_color_map[drone_id]]

        m.scale.x = 0.02

        for i in range(1, len(path)):
            m.points.append(path[i-1].position())
            m.points.append(path[i].position())

        self.__future_path_publisher.publish(m)

    @staticmethod
    def __get_pose_marker(pose, color, id, scale=1.0):
        """
        Creates an arrow marker with information from a pose.
        :param pose: Pose object to be created.
        :param color: Color of the marker.
        :param id: Id of the marker.
        :param scale: float to adjust marker size.
        :return: Arrow Marker representing the pose.
        """
        marker = Marker()
        marker.header.frame_id = str(1)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = id
        marker.color = color
        marker.lifetime = rospy.Duration(1)

        marker.scale.x = scale * MARKER_ARROW_SIZES[0]
        marker.scale.y = scale * MARKER_ARROW_SIZES[1]
        marker.scale.z = scale * MARKER_ARROW_SIZES[2]

        marker.pose = pose
        return marker

    @staticmethod
    def new_point(drone):
        # Returns a new point correspondent to where the drone is
        p = Point(drone.pose.position.x, drone.pose.position.y, drone.pose.position.z)
        return p

    def add_in_trajectory(self, drones, drone):
        # Characteristics of each marker
        line_markers = Marker()
        line_markers.header.frame_id = str(1)
        line_markers.type = line_markers.LINE_STRIP
        line_markers.action = line_markers.ADD
        line_markers.id = drone.id

        # Size of the line
        line_markers.scale.x = MARKER_LINE_SIZE_X

        # Color of the marker
        cnt = len(self.__drone_color_map)
        self.__drone_color_map[drone.id] = cnt
        line_markers.color = COLORS_T[self.__drone_color_map[drone.id]]

        # Position of the marker - we suppose here that the drone
        # has an initial position before starting to move
        line_markers.points.append(self.new_point(drone))

        # Add it in our marker array
        self.__past_path_markers.markers.append(line_markers)
        self.__drones = drones

    def remove_drone(self, drones, drone_id=0):
        if drone_id == 0:
            for i in range(len(self.__past_path_markers)):
                del self.__past_path_markers[i]
        else:
            for i in range(len(self.__past_path_markers)):
                if self.__past_path_markers[i].id == drone_id:
                    del self.__past_path_markers[i]
        self.__drones = drones
