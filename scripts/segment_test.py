import random
from copy import copy

from representations.Point import Point
from representations.Segment import Segment
from representations.Constants import EPS


def approx_dist(s1, s2, pts_per_seg=100):
    """
    Calculates approximate minimum and maximum distances between two segments by iterating through
    a number of points inside them.
    :param s1: First segment.
    :param s2: Second segment.
    :param pts_per_seg: Number of points the segments will be divided in.
    :return: Tuple(float, float) with minimum and maximum distances.
    """
    d1 = (s1.b - s1.a) / (pts_per_seg - 1)
    d2 = (s2.b - s2.a) / (pts_per_seg - 1)
    min_dist = float("inf")
    max_dist = float("-inf")

    p1 = copy(s1.a)
    for i in range(pts_per_seg):
        p2 = copy(s2.a)
        for j in range(pts_per_seg):
            min_dist = min(min_dist, p1.dist(p2))
            max_dist = max(max_dist, p1.dist(p2))
            p2 += d2
        p1 += d1
    return min_dist, max_dist


if __name__ == '__main__':
    N = 100
    TEST_DISTANCES = True
    TEST_POINTS_WITH_DISTANCE = True
    # random.seed(2)  # For determinism

    for k in range(N):
        def random_point():
            return Point(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))


        p1 = random_point()
        p2 = random_point()
        p3 = random_point()
        p4 = random_point()
        s1 = Segment(p1, p2)
        s2 = Segment(p3, p4)

        if TEST_DISTANCES:
            approx = approx_dist(s1, s2)
            if s1.min_distance(s2) - approx[0] > 0.02:
                print("======== MIN DIST ERROR ========")
                print("Seg1: ", s1)
                print("Seg2: ", s2)
                print("Calculated dist: ", s1.min_distance(s2), "Approximated dist: ", approx[0])
            if s1.max_distance(s2) - approx[1] > 0.02:
                print("======== MAX DIST ERROR ========")
                print("Seg1: ", s1)
                print("Seg2: ", s2)
                print("Calculated dist: ", s1.max_distance(s2), "Approximated dist: ", approx[1])
        if TEST_POINTS_WITH_DISTANCE:
            d = 1
            reg = s1.region_near_segment(s2, d)
            error = False
            if reg is None:
                error = s1.max_distance(s2) < d
                if error:
                    print("======== REGION ERROR ========")
                    print("Error1: No region, but max distance is smaller than d")
            else:
                r1, r2 = reg
                error1 = s1.min_distance(s2) > d
                error2 = not (s1.contains(r1) and s1.contains(r2))
                # Both extremes of region are near
                error3 = not (s2.dist_to_point(r1) < d + EPS and
                              s2.dist_to_point(r2) < d + EPS)
                # If r1 is not A, check if its distance is equal to d
                if r1 != s1.a:
                    error4 = abs(s2.dist_to_point(r1) - d) > EPS
                else:
                    error4 = s1.a.dist_to_segment(s2) > d
                # If r2 is not B, check if its distance is equal to d
                if r2 != s1.b:
                    error5 = abs(s2.dist_to_point(r2) - d) > EPS
                else:
                    error5 = s1.b.dist_to_segment(s2) > d
                # Min dist is still inside region
                error6 = abs(Segment(r1, r2).min_distance(s2) - s1.min_distance(s2)) > EPS
                error = error1 | error2 | error3 | error4 | error5 | error6

                if error:
                    print("======== REGION ERROR ========")
                    if error1:
                        print("Error1: Min distance larger than d")
                    if error2:
                        print("Error2: Segment doesn't contain r1 or r2")
                    if error3:
                        print("Error3: r1 or r2 are too far away from s2")
                    if error4:
                        print("Error4: r1 bad placed")
                    if error5:
                        print("Error5: r2 bad placed")
                    if error6:
                        print("Error6: Min dist is not in region")
            if error:
                print("Seg1: ", s1)
                print("Seg2: ", s2)
                print("Max dist: ", s1.max_distance(s2), " Min dist: ", s1.min_distance(s2))
                print("Region: ", reg)
                if reg is not None:
                    print("Reg dist: ", reg[0].dist_to_segment(s2), reg[1].dist_to_segment(s2))
