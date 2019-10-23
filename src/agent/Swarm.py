import rospy
from agent.Crazyflie import Crazyflie
from decision_making.DecisionMaking import DecisionMaking
from perception.Perception import Perception
from representations.Constants import RATE
from tools.SwarmController import SwarmController
from tools.telemetry.VisualizationPublisher import VisualizationPublisher


class Swarm:
    def __init__(self, drone_ids):
        self.__node = rospy.init_node('swarm')
        self.__drones = [Crazyflie(id) for id in drone_ids]
        self.__controller = SwarmController(self.__drones)
        self.__decision_making = DecisionMaking(self.__drones)
        self.__perception = Perception()
        self.__visualization_publisher = VisualizationPublisher()

    def run_controller(self):
        self.__controller.execute()
        r = rospy.Rate(RATE)

        while not rospy.is_shutdown():
            obstacle_collection = self.__perception.perceive()
            self.__decision_making.decide(obstacle_collection, self.__controller.get_goal())
            if not self.__controller.is_paused():
                for drone in self.__drones:
                    # TODO
                    pass
            # Call of a method in visualization_publisher that creates the interface
            self.__visualization_publisher.visualize()
            r.sleep()

    def get_drones(self):
        return self.__drones
