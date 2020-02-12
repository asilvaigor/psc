import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

from Polynomial4D import Polynomial4D
from representations import Constants


class Trajectory:
    """
    Trajectory composed by a list of polynomials. Each of these has a duration, which sum up to the
    trajectory's duration. Each of these polynomials are of degree 3, to be used in spline.
    """
    def __init__(self, path=None):
        self.__polynomials = []
        self.__duration = 0

        if path is not None:
            self.create_trajectory(path)

    @property
    def polynomials(self):
        return self.__polynomials

    @property
    def duration(self):
        return self.__duration

    def create_trajectory(self, path):
        """
        Generates a trajectory given a path. Does it using splines for each of the x, y, z
        coordinates. TODO: Currently ignores yaw, setting it to 0.
        :param path: Path object, indicating a path for the drone to follow.
        """
        self.__duration = path.times[-1]

        # Generates splines for x, y and z, using the t array as knots.
        x_spline = CubicSpline(path.times, [p.position().x for p in path.poses], bc_type='natural')
        y_spline = CubicSpline(path.times, [p.position().y for p in path.poses], bc_type='natural')
        z_spline = CubicSpline(path.times, [p.position().z for p in path.poses], bc_type='natural')

        # Passing to the structure ros will read. Note that the polynomials must have 8 constants.
        for i in range(1, len(path.times)):
            x_coef = np.concatenate((x_spline.c[:, i-1][::-1], [0] * 4))
            y_coef = np.concatenate((y_spline.c[:, i-1][::-1], [0] * 4))
            z_coef = np.concatenate((z_spline.c[:, i-1][::-1], [0] * 4))
            p = Polynomial4D(path.times[i]-path.times[i-1], x_coef, y_coef, z_coef, [0] * 8)
            self.__polynomials.append(p)

    def eval(self, t):
        """
        Evaluates the value of the trajectory at a given time, inside the trajectory's duration.
        :param t: Float indicating time. Must be less that trajectory's duration.
        :return: TrajectoryOutput variable with value at the given time.
        """
        assert t >= 0
        assert t <= self.__duration

        current_t = 0.0
        for p in self.__polynomials:
            if t < current_t + p.duration:
                return p.eval(t - current_t)
            current_t = current_t + p.duration
        return self.__polynomials[-1].eval(self.__polynomials[-1].duration)
