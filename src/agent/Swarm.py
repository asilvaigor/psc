import rospy
import time
import threading

from agent.Crazyflie import Crazyflie
from decision_making.DecisionMaking import DecisionMaking
from perception.Perception import Perception
from representations.Constants import RATE
from representations.obstacles.ObstacleCollection import ObstacleCollection
from tools.visualization.VisualizationPublisher import VisualizationPublisher


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
        self.__goal_poses = None
        self.__obstacle_collection = ObstacleCollection()
        self.__decision_making = DecisionMaking(self.__drones)
        self.__perception = Perception()
        self.__visualization_publisher = VisualizationPublisher(self.__drones)
        self.__lock = threading.Lock()
        self.__terminated = False

    def run_thread(self):
        """
        Creates a thread and runs the drone's pipeline. It detects obstacles, decides trajectory
        and updates visualizer. The thread will run until stop_thread is called.
        Edit: not deciding trajectory here anymore, because of time. Doing it in the unpause button.
        """
        def pipeline():
            # Publishing first things to rviz
            time.sleep(0.5)  # FIXME: for some reason the obstacles/mesh weren't being published
            # here. Perhaps something to do with ros initialization?
            self.__obstacle_collection = self.__perception.perceive()
            self.__decision_making.decide(self.__obstacle_collection, self.__goal_poses)
            self.__visualization_publisher.visualize_world(self.__decision_making.mesh,
                                                           self.__obstacle_collection)

            while not rospy.is_shutdown() and not self.__terminated:
                cur_t = time.time()
                with self.__lock:
                    # obstacle_collection = self.__perception.perceive()
                    # self.__decision_making.decide(obstacle_collection)
                    self.__visualization_publisher.visualize(self.__goal_poses)
                t_remaining = max(0, 1.0 / RATE - (time.time() - cur_t))
                time.sleep(t_remaining)

        t = threading.Thread(target=pipeline)
        t.start()

    @property
    def drones(self):
        """
        Getter for drone dict.
        :return: Dict of Crazyflie's for each drone_id.
        """
        return self.__drones

    def stop_thread(self):
        """
        Stops the swarm thread from running.
        """
        with self.__lock:
            self.__terminated = True

    def set_goal_poses(self, goal_poses):
        """
        Sets goal poses.
        :param goal_poses: Dict of goal StablePoses for each drone in the trajectory planner.
        """
        self.__goal_poses = goal_poses

    def unpause(self):
        """
        Unpause all the drones, making them move autonomously again. Note that the drones will
        initialize paused.
        """
        with self.__lock:
            self.__obstacle_collection = self.__perception.perceive()
            self.__decision_making.unpause(self.__obstacle_collection, self.__goal_poses)
            self.__visualization_publisher.visualize_world(self.__decision_making.mesh,
                                                           self.__obstacle_collection)
            self.__visualization_publisher.visualize_paths()

    def pause(self):
        """
        Pauses all the drones. Their motors will still be running and they will be stabilized in
        their current position. Note that the drones will initialize paused.
        """
        with self.__lock:
            self.__decision_making.pause()

    def shutdown_drone(self, drone_id=0):
        """
        Completely stops a drone, killing its motors.
        :param drone_id: Drone to be stopped.
        """
        with self.__lock:
            if drone_id == 0:
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
        with self.__lock:
            self.__decision_making.goto_drone(drone_id, pose)

    def add_drone(self, drone_id):
        """
        Adds a drone to the dict of used drones.
        :param drone_id: Id of the new drone.
        """
        with self.__lock:
            self.__drones[drone_id] = Crazyflie(drone_id)
            self.__visualization_publisher.add_in_trajectory(self.__drones, self.__drones[drone_id])

    def remove_drone(self, drone_id=0):
        """
        Removes a drone from the dict of used drones.
        :param drone_id: Id of the drone to be removed.
        """
        with self.__lock:
            if drone_id == 0:
                self.__drones.clear()
            else:
                del self.__drones[drone_id]
            self.__visualization_publisher.remove_drone(self.__drones, drone_id)