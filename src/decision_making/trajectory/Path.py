from representations.Constants import MAX_VEL
from representations.Point import Point


class Path:
    """
    Describes a path for the drones with a time schedule to reach each node.
    """
    def __init__(self, poses=None, times=None):
        """
        Default constructor.
        :param poses: List of StablePose indicating the pose at each point of the path.
        :param times: List of floats, indicating the time the drone should reach each of the poses,
        in respect to the time at the start of the path.
        """
        self.__poses = []
        self.__times = []
        self.__intersections = []
        self.__lengths = []
        if poses is not None:
            for i in range(len(poses)):
                if times is None or i >= len(times):
                    self.add_pose(poses[i])
                else:
                    self.add_pose(poses[i], times[i])

    @property
    def poses(self):
        return self.__poses

    @property
    def times(self):
        return self.__times

    @property
    def length(self):
        return self.__lengths[-1]

    @property
    def intersections(self):
        return self.__intersections

    def add_pose(self, pose, time=None):
        """
        Adds a new point to the path.
        :param pose: Pose of the drone.
        :param time: Time the drone should reach this pose, in respect to the initial pose in the
        path.
        """
        self.__poses.append(pose)
        if time is not None:
            self.__times.append(time)
        else:
            if len(self.__poses) == 1:
                self.__times.append(0)
                self.__lengths.append(0)
            else:
                duration = self.__poses[-1].dist(self.__poses[-2]) / MAX_VEL
                self.__times.append(self.__times[-1] + duration)
                self.__lengths.append(self.__lengths[-1] +
                                      Point(self.__poses[-2].position()).dist(
                                          Point(self.__poses[-1].position())))

    def add_intersection(self, intersection):
        """
        Adds points where the path collides with other paths. Only used for visualization.
        :param intersection: Tuple of floats, with distances at start and end of the region, in
        respect to the beginning of the path.
        """
        self.__intersections.append(intersection)

    def length_until(self, i):
        """
        Returns the total distance in the path, until a certain pose.
        :param i: Index of the pose.
        :return: float
        """
        return self.__lengths[i]

    def get_lengths(self):
        """
        Returns a list with the distances from the start to each of the poses.
        :return: List of float
        """
        return self.__lengths

    def __repr__(self):
        s = "Poses: ["
        for i in range(len(self.__poses) - 1):
            s += Point(self.__poses[i].position()).__repr__() + " -> "
        s += Point(self.__poses[-1].position()).__repr__() + "]"

        s += "\nTimes: "
        for i in range(len(self.__times) - 1):
            s += str(self.__times[i]) + " -> "
        s += str(self.__times[-1]) + "]"

        return s
