from geometry_msgs.msg import Pose

from agent.Crazyflie import Crazyflie
from decision_making.AStarPlanner import AStarPlanner


class DecisionMaking:
    """
    Main class for decision making. The goal is, with information for all the drones and obstacles
    positions, as well as a goal, create a trajectory of each robot.
    """

    def __init__(self, drones):
        """
        Constructor, which initializes communication services.
        :param drones: Dict of used drones.
        """
        self.__drones = drones
        self.__goal_poses = {}
        self.__is_paused = True

    def decide(self, obstacle_collection):
        """
        Main loop function for decision making. It calculates the best trajectories for each drone
        to reach the goal avoiding obstacles, and sets them on the drones.
        :param obstacle_collection: Obstacles in the world.
        """

        if not self.__is_paused:
            pass

    def unpause(self, goal_poses):
        """
        Unpause all the drones, calculating the new trajectory and making them move autonomously
        again. Note that the drones will initialize paused.
        @param goal_poses: Dict of StablePose objects, for the goal pose for each drone_id.
        """
        self.__goal_poses = goal_poses
        self.__is_paused = False

    def pause(self):
        """
        Pauses all the drones. Their motors will still be running and they will be stabilized in
        their current position. Note that the drones will initialize paused.
        """
        for drone_id in self.__drones:
            self.__drones[drone_id].pause()
        self.__is_paused = True

    def goto_drone(self, drone_id, pose):
        """
        Moves a drone to a given position in a straight line.
        :param drone_id: Drone to be moved.
        :param pose: Desired pose.
        """
        if self.__is_paused:
            self.__drones[drone_id].goto(pose)

    def stop_drone(self, drone_id):
        """
        Completely stops a drone, killing its motors.
        :param drone_id: Drone to be stopped.
        """
        if self.__is_paused and drone_id in self.__drones:
            self.__drones[drone_id].stop()
