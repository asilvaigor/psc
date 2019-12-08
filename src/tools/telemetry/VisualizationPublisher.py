import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from representations.Constants import COLORS
from representations.Constants import COLORS_T
from representations.Constants import MARKER_ARROW_SIZES
from representations.Constants import MARKER_LINE_SIZE_X


class VisualizationPublisher:

    def __init__(self, drones):
        """
        Basic constructor
        """
        self.__drones = drones
        self.__trajectory_markers = MarkerArray()
        for drone in self.__drones.values():
            self.add_in_trajectory(drone)

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
        drone_marker.header.frame_id = str(drone.id)
        drone_marker.type = drone_marker.ARROW
        drone_marker.action = drone_marker.ADD

        # Size of the marker
        drone_marker.scale.x = MARKER_ARROW_SIZES[0]
        drone_marker.scale.y = MARKER_ARROW_SIZES[1]
        drone_marker.scale.z = MARKER_ARROW_SIZES[2]

        # Color of the marker
        drone_marker.color = COLORS[drone.id - 1]

        # Position of the marker
        drone_marker.pose = drone.pose
        return drone_marker

    def __update_trajectory(self):
        # For each drone we add his position in the moment
        for m in self.__trajectory_markers.markers:
            m.points.append(self.__new_point(self.__drones[int(m.header.frame_id)]))

    def __new_point(self, drone):
        # Returns a new point correspondent to where the drone is
        p = Point(drone.pose.position.x, drone.pose.position.y, drone.pose.position.z)
        return p

    def add_in_trajectory(self, drone):
        # Characteristics of each marker
        line_markers = Marker()
        line_markers.header.frame_id = str(drone.id)
        line_markers.type = line_markers.LINE_STRIP
        line_markers.action = line_markers.ADD

        # Size of the line
        line_markers.scale.x = MARKER_LINE_SIZE_X

        # Color of the marker
        line_markers.color = COLORS_T[drone.id - 1]

        # Position of the marker - we suppose here that the drone
        # has an initial position before starting to move
        p = Point(drone.pose.position.x, drone.pose.position.y, drone.pose.position.z)
        line_markers.points.append(p)

        # Add it in our marker array
        self.__trajectory_markers.markers.append(line_markers)

    def remove_drone(self, drone_id=0):
        for m in self.__trajectory_markers.markers:
            if drone_id == 0 or int(m.header.frame_id) == drone_id:
                del m
