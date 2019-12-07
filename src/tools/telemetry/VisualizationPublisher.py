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
        dronemarker = Marker()
        dronemarker.header.frame_id = str(drone.id)
        dronemarker.type = dronemarker.ARROW
        dronemarker.action = dronemarker.ADD

        #drone_trajectory.header.frame_id = Marker();
        #drone_trajectory = Marker();
        #drone_trajectory = Marker();
        #drone_trajectory = Marker();

        # Size of the marker
        dronemarker.scale.x = MARKER_SIZES[0]
        dronemarker.scale.y = MARKER_SIZES[1]
        dronemarker.scale.z = MARKER_SIZES[2]

        # Color of the marker
        dronemarker.color = COLORS[drone.id - 1]

        # Position of the marker
        dronemarker.pose = drone.pose
        return dronemarker
