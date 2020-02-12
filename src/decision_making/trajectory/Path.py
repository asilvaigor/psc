from representations.Constants import MAX_VEL


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
            else:
                duration = self.__poses[-1].dist(self.__poses[-2]) / MAX_VEL
                self.__times.append(self.__times[-1] + duration)
