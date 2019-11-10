import rospy

from agent.Crazyflie import Crazyflie
from decision_making.DecisionMaking import DecisionMaking
from perception.Perception import Perception
from representations.Constants import RATE
from tools.telemetry.VisualizationPublisher import VisualizationPublisher


class Swarm:
    """
    Defines a swarm of drones. It supports obstacle detection, drone trajectory planning and
    visualization. The number of drones can be changed during runtime. This class is supposed to be
    used with the SwarmController tool, but can also be used independently.
    """

    def __init__(self, drone_ids=None):
        """
        Constructor which initializes drones, decision_making, perception and the visualization
        tool. Note that drones will initialize paused.
        :param drone_ids: List with ids of drones to be used. This list of drones can be changed
        on runtime.
        """

        # drone_ids is mutable
        if drone_ids is None:
            drone_ids = []

        self.__drones = {}
        for i in drone_ids:
            self.__drones[i] = Crazyflie(i)
        self.__decision_making = DecisionMaking(self.__drones)
        self.__perception = Perception()
        self.__visualization_publisher = VisualizationPublisher(self.__drones)

    def update(self):
        """
        Main loop for the swarm. It detects obstacles, decides trajectory and updates visualizer.
        """

        r = rospy.Rate(RATE)

        while not rospy.is_shutdown():
            obstacle_collection = self.__perception.perceive()
            self.__decision_making.decide(obstacle_collection)
            self.__visualization_publisher.visualize()
            r.sleep()

    def unpause(self, goal_pose):
        """
        Unpause all the drones, making them move autonomously again. Note that the drones will
        initialize paused.
        @param goal_pose: Goal pose in the trajectory planner.
        """
        self.__decision_making.unpause(goal_pose)

    def pause(self):
        """
        Pauses all the drones. Their motors will still be running and they will be stabilized in
        their current position. Note that the drones will initialize paused.
        """
        self.__decision_making.pause()

    def shutdown_drone(self, drone_id=0):
        """
        Completely stops a drone, killing its motors.
        :param drone_id: Drone to be stopped.
        """
        if drone_id == 0:
            print("shutting")
            for key in self.__drones.keys():
                self.__decision_making.stop_drone(key)
        else:
            self.__decision_making.stop_drone(drone_id)

    def goto_drone(self, drone_id, pose):
        """
        Moves a drone to a given position in a straight line.
        :param drone_id: Drone to be moved.
        :param pose: Desired pose.
        """
        self.__decision_making.goto_drone(drone_id, pose)

    def add_drone(self, drone_id):
        """
        Adds a drone to the dict of used drones.
        :param drone_id: Id of the new drone.
        """
        self.__drones[drone_id] = Crazyflie(drone_id)

    def remove_drone(self, drone_id):
        """
        Removes a drone from the dict of used drones.
        :param drone_id: Id of the drone to be removed.
        """
        del self.__drones[drone_id]
