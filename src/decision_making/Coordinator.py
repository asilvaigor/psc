from MeshNode import MeshNode

class Coordinator:
    def __init__(self):
        pass

    def coordinate(self, paths):
        """

        :param paths:
        :return: List of times
        """
        deltas = self.__calculate_deltas(paths)
        return []

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

    def __compute_monotone_coordination(self, paths, deltas, drone_id_1, drone_id_2):
        pass
