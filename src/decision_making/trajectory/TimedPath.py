class TimedPath:
    """
    Describes a path for the drones with a time schedule to reach each node.
    """
    def __init__(self, path, times):
        self.__path = []
        self.__times = []

