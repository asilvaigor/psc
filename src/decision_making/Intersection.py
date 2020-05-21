class Intersection:
    def __init__(self, interval_1, interval_2, orientation=0):
        """
        :param interval_1: Tuple of floats, corresponding to where the intersection begins and ends
        on the first path. Each float is the distance to the beginning of the path.
        :param interval_2: Tuple of floats, corresponding to where the intersection begins and ends
        on the second path. Each float is the distance to the beginning of the path.
        :param orientation: 1 for clockwise and -1 for counterclockwise.
        """
        self.__orientation = orientation
        self.__interval_1 = interval_1
        self.__interval_2 = interval_2

    @property
    def interval_1(self):
        """
        :return: Tuple of floats, corresponding to where the intersection begins and ends
        on the first path. Each float is the distance to the beginning of the path.
        """
        return self.__interval_1

    @property
    def interval_2(self):
        """
        :return: Tuple of floats, corresponding to where the intersection begins and ends
        on the second path. Each float is the distance to the beginning of the path.
        """
        return self.__interval_2

    @property
    def orientation(self):
        """
        :return: 1 for clockwise and -1 for counterclockwise.
        """
        return self.__orientation

    def set_orientation(self, orientation):
        assert(orientation == 1 or orientation == -1)
        self.__orientation = orientation

    def __repr__(self):
        return self.interval_1.__repr__() + "/" + self.interval_2.__repr__()
