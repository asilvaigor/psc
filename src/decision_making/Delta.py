class Delta:
    def __init__(self, nodes_paths):
        # """
        #
        #         :param paths:
        #         :return: Dict mapping tuple (i, j) of two drone_ids to array of collision regions, which are
        #         tuples of two tuples of Points.
        # """
        # # We calculate the size of the path for each drone so we can calculate the proportions after
        # trajectory_size = {}
        # for drone_id in paths:
        #     path = paths[drone_id]
        #     distance = 0
        #     ant = path[0]
        #     for i in range(1, len(path)):
        #         distance += path[i].dist(ant)
        #         ant = path[i]
        #     trajectory_size[drone_id] = distance
        #
        # # First we create the deltas that we will return
        # deltas = {}
        # for i in paths:
        #     for j in paths:
        #         if j == i:
        #             continue
        #         deltas[(i, j)] = self.__compute_delta(paths, i, j, trajectory_size[i],
        #                                               trajectory_size[j])
        #
        # return deltas

        # https://math.stackexchange.com/questions/27559/finding-points-on-two-linear-lines-which-are-a-particular-distance-apart

        pass

    def get_collisions(self, i, j):
        pass

    def get_critical_events(self, i, j):
        pass
