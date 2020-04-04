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
                    self[i, j] = []
                    for intersection in self.__deltas[j, i]:
                        self[i, j].append(Intersection(intersection.interval_2,
                                                       intersection.interval_1,
                                                       -intersection.orientation))
                else:
                    self[i, j] = self.__compute_intersections(paths[i], paths[j])

    @staticmethod
    def __compute_intersections(path_1, path_2):
        """
        Computes the regions of intersection between two paths.
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

            for i1 in intersections:
                if i1.orientation == 2:
                    continue
                for i2 in intersections:
                    if intersections.index(i2) <= intersections.index(i1) or i2.orientation == 2:
                        continue

                    intersect_1 = i1.interval_1[0] - EPS < i2.interval_1[0] < \
                                  i1.interval_1[1] + EPS or i1.interval_1[0] - EPS < \
                                  i2.interval_1[1] < i1.interval_1[1] + EPS
                    intersect_2 = i1.interval_2[0] - EPS < i2.interval_2[0] < \
                                  i1.interval_2[1] + EPS or i1.interval_2[0] - EPS < \
                                  i2.interval_2[1] < i1.interval_2[1] + EPS
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
        # TODO: A*

        return intersections

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
