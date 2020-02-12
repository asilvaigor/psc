from decision_making.trajectory.Path import Path


class Coordinator:
    def __init__(self):
        self.__paths = {}

    def coordinate(self, nodes_paths):
        """
        Generates coordinated paths by adjusting drone velocities given the current path, so the
        drones won't collide. This algorithm is based on the article by  Robert Ghrist, Jason
        M. O'Kane and Steven M. LaValle, intitled Computing Pareto Optimal Coordinations on
        Roadmaps, published on The International Journal of Robotics Research, year 2005.
        :param nodes_paths: List of MeshNode objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        # deltas = self.__compute_deltas(nodes_paths)

        for drone_id in nodes_paths:
            self.__paths[drone_id] = Path(nodes_paths[drone_id])
        return self.__paths

    def __compute_deltas(self, paths):
        """

        :param paths:
        :return: Dict of
        """
        # We calculate the size of the path for each drone so we can calculate the proportions after
        trajectory_size = {}
        for drone_id in paths:
            path = paths[drone_id]
            distance = 0
            ant = path[0]
            for i in range(1, len(path)):
                distance += path[i].dist(ant)
                ant = path[i]
            trajectory_size[drone_id] = distance

        # First we create the deltas that we will return
        deltas = {}
        for i in paths:
            for j in paths:
                if j == i:
                    continue
                deltas[(i,j)] = self.__compute_delta(paths, i, j, trajectory_size[i], trajectory_size[j])

        return deltas

    def __compute_delta(self, paths, drone_id_1, drone_id_2, traj_size_1, traj_size_2):
        """

                :param paths:
                :param drone_id_1:
                :param drone_id_2:
                :param traj_size_1:
                :param traj_size_2:
                :return: List of tuples
        """
        delta = []
        # Then, we pass by each point and search for
        # TODO
        return delta
