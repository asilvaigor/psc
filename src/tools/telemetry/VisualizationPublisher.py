import rospy
import numpy as np
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

    def visualizate(self):
        # Here we call each drone
        for i in range(len(self.__drones)):
            self.__visualizate_drone(self.__drones[i])

    def __visualizate_drone(self, drone):
        # Show each drone
        topic = ''
        publisher = rospy.Publisher(topic, )
        # TODO