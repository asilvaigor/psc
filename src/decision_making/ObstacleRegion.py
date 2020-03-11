class ObstacleRegion:
    def __init__(self, orientation, positions):
        # TODO Put the time where it should be found? Or this should be in the event?
        """
                :param orientation: 1 for clockwise and -1 for counterclockwise
                :param positions: tuple of tuples corresponding to the 4 points of the region
        """
        self.__orientation = orientation
        self.__positions = positions

    def get_positions(self):
        return self.__positions

    def get_orientation(self):
        return self.__orientation
