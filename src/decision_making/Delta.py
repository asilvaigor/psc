class Delta:
    def __init__(self, nodes_paths):
        # First we calculate the

        """
                :param paths:
                :return: Dict mapping tuple (i, j) of two drone_ids to array of collision regions, which are
               ObstacleRegion
        """
        # We calculate the size of the path for each drone so we can calculate the proportions after
        trajectory_size = {}
        for drone_id in nodes_paths:
            path = nodes_paths[drone_id]
            distance = 0
            ant = path[0]
            for i in range(1, len(path)):
                distance += path[i].dist(ant)
                ant = path[i]
            trajectory_size[drone_id] = distance

        # We create the deltas
        self.__deltas = {}
        for i in nodes_paths:
            for j in nodes_paths:
                if j == i:
                    continue
                self.__deltas[(i, j)] = self.__compute_delta(nodes_paths[i], nodes_paths[j], i, j, trajectory_size[i],
                                                      trajectory_size[j])


    def __compute_delta(self, path_i, path_j, i, j, trajectory_size_i, trajectory_size_j):
        """
                :param ???:
                :return: Array of Obstacle_regions
                Dict mapping tuple (i, j) of two drone_ids to array of collision regions, which are
               ObstacleRegion
        """
        # TODO compute each delta
        # https://math.stackexchange.com/questions/27559/finding-points-on-two-linear-lines-which-are-a-particular-distance-apart
        # TODO implement to find the points where there is a minimum distance
        pass

    def get_collisions(self, i, j):
        return self.__deltas[(i, j)]

    def get_critical_events(self, i, j):
        # TODO don't know what to do yet
        pass
