class Delta:
    def __init__(self, paths):
        """
        Calculates the regions of intersection between the paths of each drone.
        It also calculates the best orientation for each intersection (CW or CCW) by doing A* for
        every two drones.
        :param paths: Dict mapping drone_id to Path objects representing a path to follow.
        """
        self.__deltas = {}  # Dict mapping tuple (i, j) of two drone_ids to list of collision
        # regions, which are Intersection

        # Computing deltas
        for i in paths:
            for j in paths:
                if j == i:
                    continue
                if (j, i) in self.__deltas:
                    self[i, j] = self[j, i]
                else:
                    self[i, j] = self.__compute_delta(paths[i], paths[j])

    def __compute_delta(self, path_1, path_2):
        """
        Computes the regions of intersection between two paths.
        :param path_1: First path.
        :param path_2: Second path.
        :return: List of Intersection objects.
        """
        raise NotImplementedError

    def __getitem__(self, item):
        """
        Accesses __deltas.
        :param item: Tuple of ints with two drone_ids.
        :return: List of Intersection objects.
        """
        assert(len(item) == 2)
        return self.__deltas[item]

    def __setitem__(self, item, delta):
        """
        Sets __deltas.
        :param item: Tuple of ints with two drone_ids.
        :param delta: List of Intersection objects.
        """
        self.__deltas[item] = delta
