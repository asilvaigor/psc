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

    def visualize(self):
        topic = 'visualization_drones'
        publisher = rospy.Publisher(topic, MarkerArray)
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

        # Size of the marker
        dronemarker.scale.x = 0.1
        dronemarker.scale.y = 0.1
        dronemarker.scale.z = 0.1

        # Color of the marker
        dronemarker.color = COLORS[drone.id - 1]

        # Position of the marker
        dronemarker.pose = drone.pose
        return dronemarker
