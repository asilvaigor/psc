import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from representations.Constants import COLORS
from representations.Constants import MARKER_SIZES

class VisualizationPublisher:

    def __init__(self, drones):
        """
        Basic constructor
        """
        self.__drones = drones

    def visualize(self):
        topic = 'visualization_drones'
        # Review the queue_size
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones.values():
            drones_markers.markers.append(self.__visualize_drone(drone))

        # Publish the array of markers

        publisher.publish(drones_markers)

    def __visualize_drone(self, drone):
        # Show each drone
        drone_marker = Marker()
        drone_marker.header.frame_id = str(drone.id)
        drone_marker.type = drone_marker.ARROW
        drone_marker.action = drone_marker.ADD

        # Size of the marker
        drone_marker.scale.x = MARKER_SIZES[0]
        drone_marker.scale.y = MARKER_SIZES[1]
        drone_marker.scale.z = MARKER_SIZES[2]

        # Color of the marker
        drone_marker.color = COLORS[drone.id - 1]

        # Position of the marker
        drone_marker.pose = drone.pose
        return drone_marker
