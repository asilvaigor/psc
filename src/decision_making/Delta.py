import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines

from decision_making.AStarPlanner import AStarPlanner
from decision_making.DeltaNode import DeltaNode
from decision_making.Intersection import Intersection
from representations.Constants import EPS
from representations.Segment import Segment
from representations.Point import Point


class Delta:
    def __init__(self, paths):
        """
        Calculates the regions of intersection between the paths of each drone.
        It also calculates the best orientation for each intersection (CW or CCW) by doing A* for
        every two drones.
        O(N^2 * (ns^2 + i^3)), where N: number of paths,
                                     ns: maximum number of segments in a path,
                                     i: maximum number of intersections in a path.
        :param paths: Dict mapping drone_id to Path objects representing a path to follow.
        """
        self.__deltas = {}  # Dict mapping tuple (i, j) of two drone_ids to list of collision
        # regions, which are Intersection
        self.__paths = paths
        self.__delta_path = {}  # Used only for visualization

        # Computing deltas
        for i in paths:
            for j in paths:
                if j == i:
                    continue
                if (j, i) in self.__deltas:
                    self[i, j] = []
                    for intersection in self.__deltas[j, i]:
                        self[i, j].append(Intersection(intersection.interval_2,
                                                       intersection.interval_1,
                                                       -intersection.orientation))
                else:
                    self[i, j] = self.__compute_intersections(paths[i], paths[j], (i, j))

    def plot(self, i, j, x=None):
        """
        Plots the delta space with matplotlib.
        :param i: Id of the first drone.
        :param j: Id of the second drone.
        :param x: Tuple of list of floats, which represents a set of points to be plotted as a path.
        """
        size_transf = 6.4 / max(self.__paths[i].length, self.__paths[j].length)
        plt.figure(1, figsize=(self.__paths[i].length * size_transf,
                               self.__paths[j].length * size_transf))
        ax = plt.gca()

        for intersection in self.__deltas[i, j]:
            x1 = intersection.interval_1[0]
            y1 = intersection.interval_2[0]
            dx = intersection.interval_1[1] - x1
            dy = intersection.interval_2[1] - y1
            ax.add_patch(patches.Rectangle((x1, y1), dx, dy, facecolor='k'))
            if intersection.orientation == 1:
                s = "CW"
            else:
                s = "CCW"
            ax.text(x1 + dx / 2.0, y1 + dy / 2.0, s,
                    horizontalalignment='center', verticalalignment='center', color="w")

        for k in range(len(self.__delta_path) - 1):
            x1 = self.__delta_path[i, j][k].x
            x2 = self.__delta_path[i, j][k+1].x
            y1 = self.__delta_path[i, j][k].y
            y2 = self.__delta_path[i, j][k+1].y
            ax.add_line(mlines.Line2D([x1, x2], [y1, y2], color="r"))

        if x is not None:
            for k in range(len(x[0]) - 1):
                ax.add_line(mlines.Line2D([x[0][k], x[0][k + 1]],
                                          [x[1][k], x[1][k + 1]], color="b"))

        ax.set_xlim([0, self.__paths[i].length])
        ax.set_ylim([0, self.__paths[j].length])
        plt.show()

    def __compute_intersections(self, path_1, path_2, ids):
        """
        Computes the regions of intersection between two paths.
        O(ns1*ns2 + i^3), where ns1: number of segments in path 1,
                                ns2: number of segments in path 2,
                                i: number of intersections.
        :param path_1: First path.
        :param path_2: Second path.
        :return: List of Intersection objects.
        """

        def convert_to_segments(path):
            s = []
            for i in range(len(path.poses) - 1):
                s.append(Segment(Point(path.poses[i].position()),
                                 Point(path.poses[i + 1].position())))
            return s

        def merge(intersections):
            l = len(intersections)
            if l < 2:
                return intersections

            # Testing every Intersection, O(n2)
            # Using orientation = 2 to assign deleted intersections!
            for i1 in intersections:
                if i1.orientation == 2:
                    continue
                for i2 in intersections:
                    if intersections.index(i2) <= intersections.index(i1) or i2.orientation == 2:
                        continue

                    # If they intersect on the first path
                    intersect_1 = i1.interval_1[0] - EPS < i2.interval_1[0] < \
                                  i1.interval_1[1] + EPS or i1.interval_1[0] - EPS < \
                                  i2.interval_1[1] < i1.interval_1[1] + EPS
                    # If they intersect on the second path
                    intersect_2 = i1.interval_2[0] - EPS < i2.interval_2[0] < \
                                  i1.interval_2[1] + EPS or i1.interval_2[0] - EPS < \
                                  i2.interval_2[1] < i1.interval_2[1] + EPS
                    # If they intersect on both paths, merge them
                    if intersect_1 and intersect_2:
                        i1.interval_1 = (min(i1.interval_1[0], i2.interval_1[0]),
                                         max(i1.interval_1[1], i2.interval_1[1]))
                        i1.interval_2 = (min(i1.interval_2[0], i2.interval_2[0]),
                                         max(i1.interval_2[1], i2.interval_2[1]))
                        i2.orientation = 2

            aux = []
            for i in intersections:
                if i.orientation != 2:
                    aux.append(i)
            return aux

        # Converts paths to segments
        segments_1 = convert_to_segments(path_1)
        segments_2 = convert_to_segments(path_2)
        intersections = []

        dist_from_start_1 = 0
        for s1 in segments_1:
            dist_from_start_2 = 0
            for s2 in segments_2:
                # Intersection in s1
                interval_1 = s1.region_near_segment(s2)

                if interval_1 is not None:
                    interval_1 = (dist_from_start_1 + s1.a.dist(interval_1[0]),
                                  dist_from_start_1 + s1.a.dist(interval_1[1]))

                # Intersection in s2
                interval_2 = s2.region_near_segment(s1)
                if interval_2 is not None:
                    interval_2 = (dist_from_start_2 + s2.a.dist(interval_2[0]),
                                  dist_from_start_2 + s2.a.dist(interval_2[1]))

                if interval_1 is not None and interval_2 is not None:
                    intersections.append(Intersection(interval_1, interval_2))
                dist_from_start_2 += s2.length()
            dist_from_start_1 += s1.length()

        # Merge the intersections
        intersections = merge(intersections)

        for i in intersections:
            path_1.add_intersection(i.interval_1)
            path_2.add_intersection(i.interval_2)

        # Calculating orientations
        self.__compute_orientations(intersections, path_1.length, path_2.length, ids)

        return intersections

    def __compute_orientations(self, intersections, l1, l2, ids):
        """
        Computes orientations (CW or CCW) for every intersection. It does this by running A* on the
        2D space of the intersections, in which each intersection generates 4 nodes.
        O(i^3), where i is the number of intersections.
        TODO: This function only works for monotone paths.
        :param intersections: List of Intersection objects.
        :param l1: Total length of the first path.
        :param l2: Total length of the second path.
        """
        # Generating nodes
        nodes = [DeltaNode(0, 0), DeltaNode(l1, l2), DeltaNode(l1, 0), DeltaNode(0, l2)]
        for i in intersections:
            for a in range(2):
                for b in range(2):
                    nodes.append(DeltaNode(i.interval_1[a], i.interval_2[b]))

        # For every two nodes, adding edge if it does not pass through any intersection (they can
        # have a point in common, though)
        for n1 in nodes:
            for n2 in nodes:
                if n1 != n2 and not n1.has_edge(n2):

                    intersect = False
                    for i in intersections:
                        s = Segment(n1.position(), n2.position())
                        p1 = Point(i.interval_1[0], i.interval_2[0])
                        p2 = Point(i.interval_1[0], i.interval_2[1])
                        p3 = Point(i.interval_1[1], i.interval_2[0])
                        p4 = Point(i.interval_1[1], i.interval_2[1])
                        s1 = Segment(p1, p2)
                        s2 = Segment(p1, p3)
                        s3 = Segment(p2, p4)
                        s4 = Segment(p3, p4)

                        # If edge intersects intersection, but not on extremity
                        intersect |= s.intersects(s1) and not s.intersects_at_extremity(s1)
                        intersect |= s.intersects(s2) and not s.intersects_at_extremity(s2)
                        intersect |= s.intersects(s3) and not s.intersects_at_extremity(s3)
                        intersect |= s.intersects(s4) and not s.intersects_at_extremity(s4)
                        # Case where it passes through the diagonal
                        intersect |= s.contains(p1) and s.contains(p4)
                        intersect |= s.contains(p2) and s.contains(p3)

                    if not intersect:
                        n1.add_edge(n2)

        # Changing the distance function. This function only allows monotonous paths, and considers
        # only the highest distance one of the two drones. This distance will be proportional to
        # time, because both drones are moving at the same time, and the one that has to fly the
        # furthest is the one that will take most time.
        # TODO: This function only allows monotone paths.
        def dist_function(node1, node2):
            if node1.x > node2.x + EPS or node1.y > node2.y + EPS:
                return float("inf")
            return max(abs(node1.x - node2.x), abs(node1.y - node2.y))

        self.__delta_path[ids] = AStarPlanner().plan(nodes[0], nodes[1], dist_function)

        # Checking sides, for each intersection
        for i in intersections:
            clock_wise = False
            for k in range(len(self.__delta_path[ids]) - 1):
                s = Segment(self.__delta_path[ids][k].position(),
                            self.__delta_path[ids][k + 1].position())
                # Using middle of the intersection
                p = Point((i.interval_1[0] + i.interval_1[1]) / 2.0,
                          (i.interval_2[0] + i.interval_2[1]) / 2.0)

                # Tracing a ray up. If it intersects the path, then the point is ccw.
                # TODO: This only works for monotone paths!!
                ray = s.intersection_with_line_2d(p, Point(0, 1))
                if ray is not None:
                    clock_wise = True
                    break

            if clock_wise:
                i.orientation = 1
            else:
                i.orientation = -1

    def __getitem__(self, item):
        """
        Accesses __deltas.
        :param item: Tuple of ints with two drone_ids.
        :return: List of Intersection objects.
        """
        assert (len(item) == 2)
        return self.__deltas[item]

    def __setitem__(self, item, delta):
        """
        Sets __deltas.
        :param item: Tuple of ints with two drone_ids.
        :param delta: List of Intersection objects.
        """
        self.__deltas[item] = delta
