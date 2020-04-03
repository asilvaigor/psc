import numpy as np

from decision_making.Delta import Delta
from decision_making.trajectory.Path import Path
from representations.StablePose import StablePose


class Coordinator:
    def __init__(self):
        pass

    def coordinate(self, paths):
        """
        Generates coordinated paths by adjusting drone velocities given the current path, so the
        drones won't collide. This algorithm is based on the article by  Robert Ghrist, Jason
        M. O'Kane and Steven M. LaValle, untitled Computing Pareto Optimal Coordinations on
        Roadmaps, published on The International Journal of Robotics Research, year 2005.
        :param paths: Dict mapping drone_id to Path objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        delta = Delta(paths)

        # Initializing vectors x and t, and x_goal
        x0 = np.empty(shape=(1, len(paths)), dtype=StablePose)
        x_goal = np.empty(shape=(1, len(paths)), dtype=StablePose)
        x = []  # List of np arrays, size n_events x n_drones
        t = []  # List size n_events
        for drone_id in paths:
            np.append(x0, paths[drone_id].poses[0])
            np.append(x_goal, paths[drone_id].poses[-1])
        x.append(x0)
        t.append(0)

        # Initializing W, a list of (i, j, a).
        # Robot i is waiting for Robot j to reach point a.
        w = []

        # Running loop: going through each event, calculating velocities and setting new poses at
        # each step
        while x[-1] != x_goal:
            v = self.maximal_velocity(x, w, delta, paths)
            e = self.next_event(x, v, delta)
            w = self.update_w(w, e, delta)
            x.append(x[-1] + v * (e.time - t[-1]))
            t.append(e.time)

        return self.to_paths(x, t, paths)

    @staticmethod
    def coordinate_stub(paths):
        """
        Uses a given path as the final path.
        :param paths: Dict mapping drone_id to Path objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        return paths

    def maximal_velocity(self, x, w, delta, paths):
        """
        Calculates the velocities for every drone, to be followed until the next event.
        :param x: List of np.array of size n_drones and dtype StablePose. Contains, for each step,
        the poses for each drone.
        :param w: List of triplets (i, j, a). It indicates that robot i is waiting for robot j to
        reach point a.
        :param delta: Delta object, containing all intersections in the space.
        :param paths: Dict of Path for each drone.
        :return: np.array of size n_drones and dtype StablePose. Represents a velocity for every
        drone.
        """
        raise NotImplementedError

    def next_event(self, x, v, delta):
        """
        Computes the next event.
        :param x: List of np.array of size n_drones and dtype StablePose. Contains, for each step,
        the poses for each drone.
        :param v: np.array of size n_drones and dtype StablePose. Contains a maximal velocity for
        each drone.
        :param delta: Delta object, containing all intersections in the space.
        :return: Event object, with the next event.
        """
        raise NotImplementedError

    def update_w(self, w, e, delta):
        """

        :param w: List of triplets (i, j, a). It indicates that robot i is waiting for robot j to
        reach point a.
        :param e: Next event, which was just calculated.
        :param delta: Delta object, containing all intersections in the space.
        :return: List of triplets (i, j, a). It indicates that robot i is waiting for robot j to
        reach point a.
        """
        raise NotImplementedError

    @staticmethod
    def to_paths(x, t, old_paths):
        """
        Converts x, which is a list of positions for every drone, into paths, which is a dict
        mapping drone_id to list of positions to that drone.
        :param x: List of lists of Poses. For every step in the trajectory, it stores poses where
        every drone must be.
        :param t: List of times for every step in the trajectory, in respect to the start time.
        :param old_paths: Dict mapping drone_id to Path objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        assert (len(x) == len(t))

        paths = {}
        i = 0
        for drone_id in old_paths:
            paths[drone_id] = Path()
            for j in range(len(x)):
                paths[drone_id].add_pose(x[j][i], t[j])
            i += 1
        return paths
