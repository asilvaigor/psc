import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from representations.Constants import COLORS

class VisualizationPublisher:

    def __init__(self, drones):
        """
        Basic constructor
        """
        self.__drones = drones
        self.__counter = 0  # counter to take different colors for the drones

    def visualize(self):
        topic = 'visualization_drones'
        publisher = rospy.Publisher(topic, MarkerArray)
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones.items():
            drones_markers.markers.append(self.__visualize_drone(drone))

        # Publish the array of markers
        publisher.publish(drones_markers)

    def __visualize_drone(self, drone):
        # Show each drone
        droneMarker = Marker()
        droneMarker.header.frame_id = str(drone.cf_id)
        droneMarker.type = droneMarker.ARROW
        droneMarker.action = droneMarker.ADD

        # Size of the marker
        droneMarker.scale.x = 0.1
        droneMarker.scale.y = 0.1
        droneMarker.scale.z = 0.1

        # Color of the marker
        droneMarker.color = COLORS[self.__counter]
        self.__counter = self.__counter + 1
        if self.__counter > 5:
            self.__counter = 0

        # Position of the marker
        droneMarker.pose = drone.pose
        return droneMarker
