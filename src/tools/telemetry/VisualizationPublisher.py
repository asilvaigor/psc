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
from representations.Constants import WHITE
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z
from representations.obstacles.Cylinder import Cylinder


class VisualizationPublisher:

    def __init__(self, drones):
        """
        Basic constructor
        """
        self.__drones = drones
        self.__trajectory_markers = MarkerArray()
        self.__drone_color_map = {}
        for drone_id in drones:
            self.add_in_trajectory(drones, drones[drone_id])

    def visualize(self, obstacle_collection=None, goal_poses=None):
        """
        Updates all markers on rviz.
        :param obstacle_collection: ObstacleCollection with obstacles in the scene
        :param goal_poses: Dict of goal Pose for each drone.
        """
        topic = 'visualization_drones'
        topic_trajectory = 'visualization_trajectory'
        # Review the queue_size
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
        publisher_trajectory = rospy.Publisher(topic_trajectory, MarkerArray, queue_size=10)
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones.values():
            drones_markers.markers.append(self.__get_pose_marker(
                drone.pose, COLORS[self.__drone_color_map[drone.id]], drone.id, 1))

        # Here we update the trajectory
        self.__update_trajectory()

        # Here we update the goal poses
        if goal_poses is not None:
            self.__update_goal_poses(goal_poses)

        # Here we update the obstacles
        if obstacle_collection is not None:
            self.__update_obstacles(obstacle_collection)

        # Publish the array of markers and the trajectory
        publisher.publish(drones_markers)
        publisher_trajectory.publish(self.__trajectory_markers)

    def __update_trajectory(self):
        # For each drone we add his position in the moment
        for m in self.__trajectory_markers.markers:
            m.points.append(self.new_point(self.__drones[m.id]))

    def __update_goal_poses(self, goal_poses):
        """
        Refreshes goal poses in rviz.
        :param goal_poses: Dict of StablePoses for each drone.
        """
        publisher = rospy.Publisher('visualization_goals', MarkerArray, queue_size=10)
        markers = MarkerArray()
        if 0 in goal_poses and len(self.__drones) % 2 == 0:
            markers.markers.append(self.__get_pose_marker(
                goal_poses[0].to_ros(), WHITE, 0, 0.7))
        for drone_id in self.__drones:
            if drone_id in goal_poses:
                markers.markers.append(self.__get_pose_marker(
                    goal_poses[drone_id].to_ros(),
                    COLORS[self.__drone_color_map[drone_id]], drone_id, 0.5))

        publisher.publish(markers)

    @staticmethod
    def __update_obstacles(obstacle_collection):
        """
        Refreshes obstacles in rviz. Currently supports only cylinders.
        :param obstacle_collection: ObstacleCollection with obstacles in the scene.
        """
        publisher = rospy.Publisher('visualization_obstacles', MarkerArray, queue_size=10)
        markers = MarkerArray()
        for obstacle in obstacle_collection.obstacles:
            if isinstance(obstacle, Cylinder):
                m = Marker()
                m.header.frame_id = str(1)
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.id = obstacle_collection.obstacles.index(obstacle)
                m.color = WHITE
                m.lifetime = rospy.Duration(1)

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

        publisher.publish(markers)

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
        self.__trajectory_markers.markers.append(line_markers)
        self.__drones = drones

    def remove_drone(self, drones, drone_id=0):
        if drone_id == 0:
            for i in range(len(self.__trajectory_markers)):
                del self.__trajectory_markers[i]
        else:
            for i in range(len(self.__trajectory_markers)):
                if self.__trajectory_markers[i].id == drone_id:
                    del self.__trajectory_markers[i]
        self.__drones = drones


"""class VisualizationPublisher:

    def __init__(self, drones):
        
        self.__drones = drones
        self.__trajectory_markers = MarkerArray()
        self.__drone_color_map = {}

    def visualize(self):
        topic = 'visualization_drones'
        topic_trajectory = 'visualization_trajectory'
        # Review the queue_size
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
        publisher_trajectory = rospy.Publisher(topic_trajectory, MarkerArray, queue_size=10)
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones.values():
            drones_markers.markers.append(self.__visualize_drone(drone))

        # Here we update the trajectory
        self.__update_trajectory()

        # Publish the array of markers and the trajectory
        publisher.publish(drones_markers)
        publisher_trajectory.publish(self.__trajectory_markers)

    def __visualize_drone(self, drone):
        # Show each drone
        drone_marker = Marker()
        drone_marker.header.frame_id = 1
        drone_marker.type = drone_marker.ARROW
        drone_marker.action = drone_marker.ADD

        # Size of the marker
        drone_marker.scale.x = MARKER_ARROW_SIZES[0]
        drone_marker.scale.y = MARKER_ARROW_SIZES[1]
        drone_marker.scale.z = MARKER_ARROW_SIZES[2]

        # Color of the marker
        drone_marker.color = COLORS[self.__drone_color_map[drone.id]]

        # Position of the marker
        drone_marker.pose = drone.pose
        return drone_marker

    def __update_trajectory(self):
        # For each drone we add his position in the moment
        # for m, d in zip(self.__trajectory_markers.markers, self.__drones):
        #     m.points.append(self.__new_point(d))
        for m in self.__trajectory_markers.markers: # change fram_id
            m.points.append(self.__new_point(self.__drones[int(m.header.frame_id)]))

    def __new_point(self, drone):
        # Returns a new point correspondent to where the drone is
        p = Point(drone.pose.position.x, drone.pose.position.y, drone.pose.position.z)
        return p

    def __add_in_trajectory(self, drone):
        # Characteristics of each marker
        line_markers = Marker()
        line_markers.header.frame_id = 1
        line_markers.type = line_markers.LINE_STRIP
        line_markers.action = line_markers.ADD

        # Size of the line
        line_markers.scale.x = MARKER_LINE_SIZE_X

        # Color of the marker
        cnt = len(self.__drone_color_map)
        self.__drone_color_map[drone.id] = cnt
        line_markers.color = COLORS_T[self.__drone_color_map[drone.id]]

        # Position of the marker - we suppose here that the drone
        # has an initial position before starting to move
        line_markers.points.append(self.__new_point(drone))

        # Add it in our marker array
        self.__trajectory_markers.markers.append(line_markers)

    def remove_drone(self,  drones, drone_id=0):
        pass
        # TODO: this isnt working!! Marker still exists
        # self.__drones = drones # change this!
        # for m in self.__trajectory_markers.values():
        #    if drone_id == 0 or int(m.header.frame_id) == drone_id:
        #        del m

    def add_drone(self,  drones, drone_id):
        # change the vector of drones
        # call add_in_trajectory
        self.__drones = drones
        self.__add_in_trajectory(self.__drones[drone_id])

    # I think that the markers would be better in the dictionary form related to the ids from the drones
    
    class VisualizationPublisher:

    def __init__(self, drones):
        
        self.__drones = drones
        self.__trajectory_markers = MarkerArray()
        for drone in self.__drones.values():
            self.add_in_trajectory(drone)
        self.__drone_color_map = {}

    def visualize(self):
        topic = 'visualization_drones'
        topic_trajectory = 'visualization_trajectory'
        # Review the queue_size
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
        publisher_trajectory = rospy.Publisher(topic_trajectory, MarkerArray, queue_size=10)
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones.values():
            drones_markers.markers.append(self.__visualize_drone(drone))

        # Here we update the trajectory
        self.__update_trajectory()

        # Publish the array of markers and the trajectory
        publisher.publish(drones_markers)
        publisher_trajectory.publish(self.__trajectory_markers)

    def __visualize_drone(self, drone):
        # Show each drone
        drone_marker = Marker()
        # drone_marker.header.frame_id = 1
        drone_marker.header.frame_id = str(drone.id)
        drone_marker.type = drone_marker.ARROW
        drone_marker.action = drone_marker.ADD

        # Size of the marker
        drone_marker.scale.x = MARKER_ARROW_SIZES[0]
        drone_marker.scale.y = MARKER_ARROW_SIZES[1]
        drone_marker.scale.z = MARKER_ARROW_SIZES[2]

        # Color of the marker
        drone_marker.color = COLORS[self.__drone_color_map[drone.id]]

        # Position of the marker
        drone_marker.pose = drone.pose
        return drone_marker

    def __update_trajectory(self):
        # For each drone we add his position in the moment
        # for m, d in zip(self.__trajectory_markers.markers, self.__drones):
        #     m.points.append(self.__new_point(d))
        for m in self.__trajectory_markers.markers:
            m.points.append(self.__new_point(self.__drones[int(m.header.frame_id)]))

    def __new_point(self, drone):
        # Returns a new point correspondent to where the drone is
        p = Point(drone.pose.position.x, drone.pose.position.y, drone.pose.position.z)
        return p

    def __add_in_trajectory(self, drone):
        # Characteristics of each marker
        line_markers = Marker()
        # line_markers.header.frame_id = 1
        line_markers.header.frame_id = str(drone.id)
        line_markers.type = line_markers.LINE_STRIP
        line_markers.action = line_markers.ADD

        # Size of the line
        line_markers.scale.x = MARKER_LINE_SIZE_X

        # Color of the marker
        cnt = len(self.__drone_color_map)
        self.__drone_color_map[drone.id] = cnt
        line_markers.color = COLORS_T[self.__drone_color_map[drone.id]]

        # Position of the marker - we suppose here that the drone
        # has an initial position before starting to move
        p = Point(drone.pose.position.x, drone.pose.position.y, drone.pose.position.z)
        line_markers.points.append(p)

        # Add it in our marker array
        self.__trajectory_markers.markers.append(line_markers)

    def remove_drone(self, drone_id=0):
        # TODO: this isnt working!! Marker still exists
        for m in self.__trajectory_markers.markers:
            if drone_id == 0 or int(m.header.frame_id) == drone_id:
                del m

    def add_drone(self, drone_id=0):
        # TODO: this isnt working!! Marker still exists
        for m in self.__trajectory_markers.markers:
            if drone_id == 0 or int(m.header.frame_id) == drone_id:
                del m
    """