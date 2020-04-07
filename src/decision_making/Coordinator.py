import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import rospy
from sensor_msgs.msg import Image

from decision_making.Delta import Delta
from decision_making.trajectory.Path import Path
from representations.Constants import EPS
from representations.Constants import MAX_VEL
from representations.Point import Point
from representations.Segment import Segment


class Coordinator:
    def __init__(self):
        self.__n_drones = 0
        self.__i_to_drone_id = {}
        self.__delta = None
        self.__paths = {}

    def coordinate(self, paths, visualize=False):
        """
        Generates coordinated paths by adjusting drone velocities given the current path, so the
        drones won't collide. This algorithm is based on the article by  Robert Ghrist, Jason
        M. O'Kane and Steven M. LaValle, untitled Computing Pareto Optimal Coordinations on
        Roadmaps, published on The International Journal of Robotics Research, year 2005.
        :param paths: Dict mapping drone_id to Path objects representing a path to follow.
        :param visualize: Boolean, If true, will plot the coordination for each pair of drones.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        self.__n_drones = len(paths)
        if self.__n_drones <= 1:
            return paths

        i = 0
        for drone_id in paths:
            self.__i_to_drone_id[i] = drone_id
            i += 1

        # Computes intersections
        self.__delta = Delta(paths)

        # Initializing vectors x and t, and x_goal
        x0 = np.zeros(self.__n_drones, dtype=float)
        x = [x0]  # List of np arrays, size n_events x n_drones
        t = [0.0]  # List size n_events
        x_goal = []
        for drone_id in paths:
            x_goal.append(paths[drone_id].length)
        x_goal = np.array(x_goal)

        # Running loop: going through each event, calculating velocities and setting new poses at
        # each step
        while np.any(np.abs(x[-1] - x_goal) > EPS):
            v = self.__maximal_velocity(x, x_goal)
            dt = self.__event_dt(x, x_goal, v)
            x.append(x[-1] + v * dt)
            t.append(t[-1] + dt)

        self.__paths = paths
        if visualize:
            self.plot(x)

        return self.__to_paths(x, t, paths)

    def plot(self, x):
        """
        Plots the delta space with matplotlib, for every pair of drones.
        TODO: put this outside coordinator.
        :param x: List of np.array of size n_drones and dtype float. Contains, for each step,
        the position for each drone in the coordination space.
        """
        n_axis = int(len(self.__delta) / 2)
        n_cols = int(math.ceil(math.sqrt(n_axis)))
        n_rows = int(math.ceil(1.0 * n_axis / n_cols))

        fig, ax = plt.subplots(n_rows, n_cols, squeeze=False)
        k = 0
        for i in range(self.__n_drones):
            for j in range(self.__n_drones):
                if j <= i:
                    continue

                i_ax = k // n_cols
                j_ax = k % n_cols
                i_drone = self.__i_to_drone_id[i]
                j_drone = self.__i_to_drone_id[j]

                ax[i_ax][j_ax].set_xlim([0, self.__paths[i_drone].length])
                ax[i_ax][j_ax].set_ylim([0, self.__paths[j_drone].length])
                ax[i_ax][j_ax].set_xlabel("drone " + str(i_drone))
                ax[i_ax][j_ax].set_ylabel("drone " + str(j_drone))

                for intersection in self.__delta[i_drone, j_drone]:
                    x1 = intersection.interval_1[0]
                    y1 = intersection.interval_2[0]
                    dx = intersection.interval_1[1] - x1
                    dy = intersection.interval_2[1] - y1
                    ax[i_ax][j_ax].add_patch(patches.Rectangle((x1, y1), dx, dy, facecolor='k'))
                    if intersection.orientation == 1:
                        s = "CW"
                    else:
                        s = "CCW"
                    ax[i_ax][j_ax].text(x1 + dx / 2.0, y1 + dy / 2.0, s,
                                        horizontalalignment='center',
                                        verticalalignment='center', color="w")

                delta_path = self.__delta.get_precalculated_path()[i_drone, j_drone]
                for z in range(len(delta_path) - 1):
                    x1 = delta_path[z].x
                    x2 = delta_path[z+1].x
                    y1 = delta_path[z].y
                    y2 = delta_path[z+1].y
                    ax[i_ax][j_ax].add_line(mlines.Line2D([x1, x2], [y1, y2], color="r"))

                for z in range(len(x) - 1):
                    ax[i_ax][j_ax].add_line(mlines.Line2D([x[z][i], x[z + 1][i]],
                                                          [x[z][j], x[z + 1][j]], color="b"))

                k += 1

        try:
            fig.canvas.draw()
            data = list(np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep=''))
            h = fig.canvas.get_width_height()[1]
            w = fig.canvas.get_width_height()[0]
            img_publisher = rospy.Publisher('delta_space', Image, queue_size=10, latch=True)
            img = Image(height=h, width=w, data=data, encoding="rgb8", step=3 * w)
            img_publisher.publish(img)
        except rospy.ROSException:
            plt.show()

    @staticmethod
    def coordinate_stub(paths):
        """
        Uses a given path as the final path.
        :param paths: Dict mapping drone_id to Path objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        return paths

    def __maximal_velocity(self, x, x_goal):
        """
        Calculates the velocities for every drone, to be followed until the next event.
        O(n_drones + i), where i is the total number of intersections.
        :param x: List of np.array of size n_drones and dtype float. Contains, for each step,
        the position for each drone in the coordination space.
        :param x_goal: np.array of size n_drones and dtype float. Contains, for each drone, the
        final position in the coordination space.
        :return: np.array of size n_drones and dtype float. Represents a velocity for every
        drone in the coordination space.
        """
        v = np.ones(self.__n_drones, dtype=float) * MAX_VEL

        # Drones that are finished
        for i in range(self.__n_drones):
            if abs(x[-1][i] - x_goal[i]) < EPS:
                v[i] = 0.0

        # Going through each intersection
        for i in range(self.__n_drones):
            for j in range(self.__n_drones):
                if j <= i:
                    continue

                for intersection in self.__delta[self.__i_to_drone_id[i], self.__i_to_drone_id[j]]:
                    if intersection.orientation == 1:
                        # At the left edge of the interval, except the top-left corner
                        if x[-1][i] > intersection.interval_1[0] - EPS and \
                                x[-1][j] < intersection.interval_2[1] - EPS:
                            v[i] = 0
                    else:
                        # At the bottom edge of the interval, except the bottom-right corner
                        if x[-1][i] < intersection.interval_1[1] - EPS and \
                                x[-1][j] > intersection.interval_2[0] - EPS:
                            v[j] = 0

        # If all drones will be stopped, something is wrong
        if np.all(v < EPS):
            raise ValueError("No drones can move. Maybe the path is not monotonous?")

        return v

    def __event_dt(self, x, x_goal, v):
        """
        Computes the time until the next event.
        O(n_drones + i), where i is the total number of intersections.
        :param x: List of np.array of size n_drones and dtype float. Contains, for each step,
        the position for each drone in the coordination space.
        :param x_goal: np.array of size n_drones and dtype float. Contains, for each drone, the
        final position in the coordination space.
        :param v: np.array of size n_drones and dtype float. Contains a maximal velocity for
        each drone in the coordination space.
        :return: float, Time until the next event.
        """
        dt = float("inf")

        # Time to reach the goals
        for i in range(self.__n_drones):
            if v[i] > EPS:
                dt = min(dt, (x_goal[i] - x[-1][i]) / v[i])

        # For each intersection
        for i in range(self.__n_drones):
            for j in range(self.__n_drones):
                if j <= i:
                    continue

                for intersection in self.__delta[self.__i_to_drone_id[i], self.__i_to_drone_id[j]]:
                    # Tracing ray in direction (v[i], v[j])
                    pt = Point(x[-1][i], x[-1][j])
                    vec = Point(v[i], v[j])
                    if intersection.orientation == 1:
                        seg = Segment(Point(intersection.interval_1[0], 0),
                                      Point(intersection.interval_1[0], intersection.interval_2[1]))
                    else:
                        seg = Segment(Point(0, intersection.interval_2[0]),
                                      Point(intersection.interval_1[1], intersection.interval_2[0]))

                    # One drone is stopped, the event is at the end of the segment
                    if seg.contains(pt):
                        # If it is at the end of the segment, ignore
                        if not (seg.a == pt or seg.b == pt):
                            if intersection.orientation == 1:
                                dt = min(dt, (intersection.interval_2[1] - pt.y) / v[j])
                            else:
                                dt = min(dt, (intersection.interval_1[1] - pt.x) / v[i])
                    # If it reached the segment
                    elif seg.intersection_with_line_2d(pt, vec):
                        if intersection.orientation == 1:
                            dt = min(dt, (intersection.interval_1[0] - pt.x) / v[i])
                        else:
                            dt = min(dt, (intersection.interval_2[0] - pt.y) / v[j])

        if dt < EPS:
            raise ValueError("Something went wrong. Next event is current event")

        return dt

    def __to_paths(self, x, t, old_paths):
        """
        Converts x, which is a list of positions for every drone in the coordination space, into
        paths, which is a dict mapping drone_id to the Path object to follow.
        :param x: List of np.array of size n_drones and dtype float. Contains, for each step,
        the position for each drone in the coordination space.
        :param t: List of times for every step in x, in respect to the start time.
        :param old_paths: Dict mapping drone_id to Path objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """

        def merge_events(x, t):
            """
            Merges events if the drone keeps the same velocity between them.
            :param x: List of floats, positions of a drone in the coordination space.
            :param t: List of floats, time the drone should be at that position.
            :return: Tuple with merged_x, merged_t.
            """
            merged_x = [x[0]]
            merged_t = [t[0]]
            for i in range(1, len(x) - 1):
                vl = (x[i] - x[i - 1]) / (t[i] - t[i - 1])
                vr = (x[i + 1] - x[i]) / (t[i + 1] - t[i])
                if abs(vl - vr) > EPS:
                    merged_x.append(x[i])
                    merged_t.append(t[i])
            merged_x.append(x[-1])
            merged_t.append(t[-1])

            return merged_x, merged_t

        paths = {}
        for k in range(self.__n_drones):
            drone_id = self.__i_to_drone_id[k]
            lengths = old_paths[drone_id].get_lengths()
            poses = old_paths[drone_id].poses
            merged_x, merged_t = merge_events([v[k] for v in x], t)
            n_events = len(merged_x)

            paths[drone_id] = Path()
            paths[drone_id].add_pose(poses[0], 0)
            paths[drone_id].intersections = old_paths[drone_id].intersections

            i = 1  # Iterating in events
            j = 1  # Iterating in poses
            while i < n_events and j < len(poses):
                # Change velocity but keep speed constant
                if lengths[j] + EPS < merged_x[i]:
                    factor = (lengths[j] - merged_x[i - 1]) / (merged_x[i] - merged_x[i - 1])
                    dt = (merged_t[i] - merged_t[i - 1]) * factor
                    paths[drone_id].add_pose(poses[j], merged_t[i - 1] + dt)
                    j += 1
                # Change speed between of two poses: adding a pose in between
                elif merged_x[i] < lengths[j] - EPS:
                    factor = (merged_x[i] - lengths[j - 1]) / (lengths[j] - lengths[j - 1])
                    p = poses[j - 1] + (poses[j] - poses[j - 1]) * factor
                    paths[drone_id].add_pose(p, merged_t[i])
                    i += 1
                # Changing speed and pose at the same time
                else:
                    paths[drone_id].add_pose(poses[j], merged_t[i])
                    j += 1
                    i += 1

        return paths
