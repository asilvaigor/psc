import numpy as np

from decision_making.Delta import Delta
from decision_making.trajectory.Path import Path


class Coordinator:
    def __init__(self):
        pass

    def coordinate(self, nodes_paths):
        """
        Generates coordinated paths by adjusting drone velocities given the current path, so the
        drones won't collide. This algorithm is based on the article by  Robert Ghrist, Jason
        M. O'Kane and Steven M. LaValle, untitled Computing Pareto Optimal Coordinations on
        Roadmaps, published on The International Journal of Robotics Research, year 2005.
        :param nodes_paths: Dict mapping drone_id to List of MeshNode objects representing a path to
        follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        delta = Delta(nodes_paths)

        # Initializing vectors x and t, and x_goal
        x0 = []
        x_goal = []
        x = []
        t = []
        for drone_id in nodes_paths:
            x0.append(nodes_paths[drone_id][0])
            x_goal.append(nodes_paths[drone_id][-1])
        x.append(np.array(x0))
        t.append(0)
        x_goal = np.array(x_goal)

        # Initializing W, a list of (i, j, a)
        w = []

        # Running loop: going through each event, calculating velocities and setting new poses at
        # each step
        while x[-1] != x_goal:
            v = self.maximal_velocity(x, w, delta)
            e = self.next_event(x, v, delta)
            w = self.update_w(w, e, delta)
            x.append(x[-1] + v * (e.time - t[-1]))
            t.append(e.time)

        return self.to_paths(x, t, nodes_paths)

    @staticmethod
    def coordinate_stub(nodes_paths):
        """
        Uses a give path as the final path.
        :param nodes_paths: Dict mapping drone_id to List of MeshNode objects representing a path to
        follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        paths = {}
        for drone_id in nodes_paths:
            paths[drone_id] = Path(nodes_paths[drone_id])
        return paths

    def maximal_velocity(self, x, w, delta):
        pass

    def next_event(self, x, v, delta):
        pass

    def update_w(self, w, e, delta):
        pass

    @staticmethod
    def to_paths(x, t, nodes_paths):
        """
        Converts x, which is a list of positions for every drone into paths, which is a dict
        mapping drone_id to list of positions to that drone.
        :param x: List of lists of Poses. For every step in the trajectory, it stores poses where
        every drone must be.
        :param t: List of times for every step in the trajectory, in respect to the start time.
        :param nodes_paths: Dict mapping drone_id to List of MeshNode objects representing a path to
        follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        assert (len(x) == len(t))

        paths = {}
        i = 0
        for drone_id in nodes_paths:
            paths[drone_id] = Path()
            for j in range(len(x)):
                paths[drone_id].add_pose(x[j][i], t[j])
            i += 1
        return paths
