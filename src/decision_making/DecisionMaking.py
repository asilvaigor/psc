class DecisionMaking:
    """
    Main class for decision making. The goal is, with information for all the drones and obstacles positions, as well
    as a goal, create a trajectory of each robot.
    """

    def __init__(self, drones):
        """
        Constructor, which initializes communication services.
        :param drones: List of used drones.
        """
        self.__drones = drones

    def decide(self, obstacle_collection, goal):
        """
        Main loop function for decision making. It calculates the best trajectories for each drone to reach the goal
        avoiding obstacles, and sets them on the drones.
        :param obstacle_collection: Obstacles in the world.
        :param goal: Final expected position for the center of the drone swarm.
        """
        pass
