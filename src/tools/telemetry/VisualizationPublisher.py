import rospy
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from agent.Crazyflie import Crazyflie
from agent.Swarm import Swarm

class VisualizationPublisher:

    def __init__(self, drones):
        """
        Basic constructor
        """
        self.__drones = drones
        # Make the basic calls before it begins, open interface

    def visualize(self):
        topic = 'visualization_drones'
        publisher = rospy.Publisher(topic, MarkerArray)
        rospy.init_node('drones')
        drones_markers = MarkerArray()

        # Here we call each drone
        for drone in self.__drones:
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
        droneMarker.color.a = 1.0
        droneMarker.color.r = 1.0
        droneMarker.color.g = 1.0
        droneMarker.color.b = 0.0
        # Position of the marker
        droneMarker.pose = drone.pose
        return droneMarker