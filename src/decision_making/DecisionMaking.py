from decision_making.AStarPlanner import AStarPlanner
from decision_making.Coordinator import Coordinator
from decision_making.CGALMesh import CGALMesh


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
        self.__is_paused = True
        self.__planner = AStarPlanner()
        self.__coordinator = Coordinator()
        self.__mesh = None
        self.__paths = {}

    @property
    def mesh(self):
        """
        Getter for mesh, which represents the discrete space.
        :return: Mesh object.
        """
        return self.__mesh

    def decide(self, obstacle_collection, drone_poses, goal_poses):
        """
        Main loop function for decision making. It calculates the best trajectories for each drone
        to reach the goal avoiding obstacles, and sets them on the drones.
        :param obstacle_collection: Obstacles in the world.
        :param drone_poses: Dict of Pose objects, for the poses for each drone_id.
        :param goal_poses: Dict of StablePose objects, for the goal pose for each drone_id.
        """
        self.__mesh = CGALMesh()
        drone_nodes, goal_nodes = self.__mesh.discretize(obstacle_collection, drone_poses,
                                                         goal_poses)
        paths_nodes = {}
        for drone_id in drone_nodes:
            paths_nodes[drone_id] = self.__planner.plan(drone_nodes[drone_id], goal_nodes[drone_id])
        self.__paths = self.__coordinator.coordinate_stub(paths_nodes)

    def unpause(self, obstacle_collection, drone_poses, goal_poses):
        """
        Unpause all the drones, calculating the new trajectory and making them move autonomously
        again. Note that the drones will initialize paused.
        :param obstacle_collection: Obstacles in the world.
        :param drone_poses: Dict of Pose objects, for the poses for each drone_id.
        :param goal_poses: Dict of StablePose objects, for the goal pose for each drone_id.
        """
        self.__is_paused = False
        self.decide(obstacle_collection, drone_poses, goal_poses)
        for drone_id in self.__drones:
            self.__drones[drone_id].follow_path(self.__paths[drone_id])

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
