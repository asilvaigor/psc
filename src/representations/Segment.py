from representations.Point import Point
from representations.Constants import EPS
from representations.Constants import OBSTACLE_MARGIN


class Segment:
    """
    Represents a line segment AB, which is a line limited by two points.
    """

    def __init__(self, _a, _b):
        assert(isinstance(_a, Point))
        assert(isinstance(_b, Point))
        self.a = _a
        self.b = _b

    def length(self):
        """
        Length of the segment, in meters.
        :return: float, distance AB.
        """
        return self.a.dist(self.b)

    def dist_to_point(self, p):
        """
        Calculates the minimum distance to a given point.
        :param p: A point to calculate the distance.
        :return: float, distance to the point.
        """
        # Proj in line: AB * (AP . AB) / (AB . AB) + A
        ab = self.b - self.a
        ap = p - self.a
        proj = self.a + ab * ap.inner(ab) / ab.inner(ab)

        # If proj is in segment, use it to calculate distance. Else use a or b
        if abs(proj.dist(self.a) + proj.dist(self.b) - self.length()) < EPS:
            return proj.dist(p)
        else:
            return min(self.a.dist(p), self.b.dist(p))

    def contains(self, p):
        """
        Checks if the segment contains a point.
        :param p: Point
        :return: Boolean, True if the point is in the segment.
        """
        return self.dist_to_point(p) < EPS

    def intersects(self, s):
        """
        Checks if the segment intersects another.
        :param s: Segment
        :return: Boolean, True if the segments intersect.
        """
        return self.min_distance(s) < EPS

    def intersects_at_extremity(self, s):
        """
        Checks if the segment intersects another one of the other segment's endpoints.
        :param s: Segment
        :return: True if the segment contains one of the given segment's endpoints.
        """
        return self.contains(s.a) or self.contains(s.b)

    def intersection_2d(self, s):
        """
        Finds the intersection with another segment. Only works for 2d segments.
        :param s: Segment.
        :return: Point or None, representing the intersection.
        """
        num = ((s.a.x - self.a.x) * (s.b.y - s.a.y) - (s.a.y - self.a.y) * (s.b.x - s.a.x))
        den = ((self.b.x - self.a.x) * (s.b.y - s.a.y) - (self.b.y - self.a.y) * (s.b.x - s.a.x))
        if den > 0:
            i = num / den
            pt = Point(self.a.x + (self.b.x - self.a.x) * i, self.a.y + (self.b.y - self.a.y) * i)
            if s.contains(pt):
                return pt
        else:
            return None

    def intersection_with_line_2d(self, p, direction):
        """
        Finds the intersection of the segment with an semi-infinite line. Only works for 2d.
        :param p: Point the line starts.
        :param direction: Point, representing a vector of the direction of the line.
        :return: Point or None, representing the intersection.
        """
        return self.intersection_2d(Segment(p, p + direction * 1000))

    def min_distance(self, segment):
        """
        Calculates the minimum distance between two line segments.
        Code from <http://geomalgorithms.com/a07-_distance.html>
        :param segment: Another segment to calculate the distance to.
        :return: float, minimum distance between the two segments.
        """
        u = self.b - self.a
        v = segment.b - segment.a
        w = self.a - segment.a

        a = u.inner(u)
        b = u.inner(v)
        c = v.inner(v)
        d = u.inner(w)
        e = v.inner(w)
        big_d = a * c - b * b
        sc, s_n, s_d = big_d, big_d, big_d
        tc, t_n, t_d = big_d, big_d, big_d

        # compute the line parameters of the two closest points
        if big_d < EPS:  # the lines are almost parallel
            s_n = 0.0  # force using point P0 on segment S1
            s_d = 1.0  # to prevent possible division by 0.0 later
            t_n = e
            t_d = c
        else:  # get the closest points on the infinite lines
            s_n = b * e - c * d
            t_n = a * e - b * d
            if s_n < 0.0:  # sc < 0 = > the s=0 edge is visible
                s_n = 0.0
                t_n = e
                t_d = c
            elif s_n > s_d:  # sc > 1  = > the s=1 edge is visible
                s_n = s_d
                t_n = e + b
                t_d = c

        if t_n < 0.0:  # tc < 0 = > the t=0 edge is visible
            t_n = 0.0
            # recompute sc for this edge
            if -d < 0.0:
                s_n = 0.0
            elif -d > a:
                s_n = s_d
            else:
                s_n = -d
                s_d = a
        elif t_n > t_d:  # tc > 1  = > the t=1 edge is visible
            t_n = t_d
            # recompute sc for this edge
            if -d + b < 0.0:
                s_n = 0
            elif (-d + b) > a:
                s_n = s_d
            else:
                s_n = -d + b
                s_d = a

        # finally do the division to get sc and tc
        if abs(s_n) < EPS:
            sc = 0.0
        else:
            sc = s_n / s_d
        if abs(t_n) < EPS:
            tc = 0.0
        else:
            tc = t_n / t_d

        # get the difference of the two closest points point
        d_p = w + (u * sc) - (v * tc)  # =  S1(sc) - S2(tc)
        return d_p.norm()  # return the closest distance

    def max_distance(self, segment):
        """
        Calculates the maximum distance between two line segments.
        :param segment: Another segment to calculate the distance to.
        :return: float, maximum distance between the two segments.
        """
        d = max(self.a.dist(segment.a), self.a.dist(segment.b))
        d = max(d, self.b.dist(segment.a))
        d = max(d, self.b.dist(segment.b))
        return d

    def region_near_segment(self, segment, dist=OBSTACLE_MARGIN):
        """
        Calculates a region in the current segment which has distance less than a value from another
        segment. In this region, all points have a minimum distance to the other segment less than
        the given value.
        Uses a binary-search approach.
        :param segment: Another segment for reference.
        :param dist: float, Fixed distance the points should be distanced.
        :return: Tuple of Points. The first one is the start of the region and the second the end.
        Both points are in the segment.
        Note: If there is no region, returns None.
        """
        # No region on the segment is near
        if self.min_distance(segment) > dist:
            return None
        # Whole segment is near
        if self.max_distance(segment) < dist:
            return self.a, self.b
        # Parallel segments
        if abs(self.min_distance(segment) - self.max_distance(segment)) < EPS:
            return None

        # Binary search for the region
        def binary_search(l, r, left=True):
            m = (l + r) / 2
            dm = segment.dist_to_point(m)

            # Stopping condition, uses a precision of EPS
            if l.dist(r) < 2 * EPS:
                if left and segment.dist_to_point(l) < dist:
                    return l
                if not left and segment.dist_to_point(r) < dist:
                    return r
                if dm < dist:
                    return m
                if left:
                    return r
                else:
                    return l

            if dm < dist:
                if left:
                    return binary_search(l, m, left)
                else:
                    return binary_search(m, r, left)
            else:
                m_1 = m + (r - l) * EPS
                m_2 = m - (r - l) * EPS
                dm_1 = segment.dist_to_point(m_1)
                dm_2 = segment.dist_to_point(m_2)
                if dm_1 < dm_2:
                    return binary_search(m, r, left)
                else:
                    return binary_search(l, m, left)

        return binary_search(self.a, self.b, True), binary_search(self.a, self.b, False)

    def __repr__(self):
        """
        Used for printing.
        :return: String representing Segment.
        """
        return "[" + self.a.__repr__() + " -> " + self.b.__repr__() + "]"
