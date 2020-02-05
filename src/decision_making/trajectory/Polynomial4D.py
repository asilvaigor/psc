import numpy as np

from Polynomial import Polynomial
from TrajectoryOutput import TrajectoryOutput


# 4d single polynomial piece for x-y-z-yaw, includes duration.
# yaw means the angle in reference to a vertical axis
class Polynomial4D:
    def __init__(self, duration, px, py, pz, pyaw):
        self.duration = duration
        self.px = Polynomial(px)
        self.py = Polynomial(py)
        self.pz = Polynomial(pz)
        self.pyaw = Polynomial(pyaw)

    # compute and return derivative
    def derivative(self):
        return Polynomial4D(
            self.duration,
            self.px.derivative().p,
            self.py.derivative().p,
            self.pz.derivative().p,
            self.pyaw.derivative().p)

    @staticmethod
    def normalize(v):
        norm = np.linalg.norm(v)
        assert norm > 0
        return v / norm

    def eval(self, t):
        result = TrajectoryOutput()
        # flat variables
        result.pos = np.array([self.px.eval(t), self.py.eval(t), self.pz.eval(t)])
        result.yaw = self.pyaw.eval(t)

        # 1st derivative
        derivative = self.derivative()
        result.vel = np.array([derivative.px.eval(t), derivative.py.eval(t), derivative.pz.eval(t)])
        dyaw = derivative.pyaw.eval(t)

        # 2nd derivative
        derivative2 = derivative.derivative()
        result.acc = np.array([derivative2.px.eval(t), derivative2.py.eval(t), derivative2.pz.eval(t)])

        # 3rd derivative
        derivative3 = derivative2.derivative()
        jerk = np.array([derivative3.px.eval(t), derivative3.py.eval(t), derivative3.pz.eval(t)])

        thrust = result.acc + np.array([0, 0, 9.81])  # add gravity

        z_body = self.normalize(thrust)
        x_world = np.array([np.cos(result.yaw), np.sin(result.yaw), 0])
        y_body = self.normalize(np.cross(z_body, x_world))
        x_body = np.cross(y_body, z_body)

        jerk_orth_zbody = jerk - (np.dot(jerk, z_body) * z_body)
        h_w = jerk_orth_zbody / np.linalg.norm(thrust)

        result.omega = np.array([-np.dot(h_w, y_body), np.dot(h_w, x_body), z_body[2] * dyaw])
        return result
