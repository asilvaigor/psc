import numpy as np

from Polynomial4D import Polynomial4D
from decision_making.MeshNode import MeshNode


# Trajectory that composed by an array with polynomials and their respective durations
class Trajectory:
    def __init__(self):
        self.polynomials = None
        self.duration = None

    def loadcsv(self, filename):
        data = np.loadtxt(filename, delimiter=",", skiprows=1, usecols=range(33))
        self.polynomials = [Polynomial4D(row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in data]
        self.duration = np.sum(data[:, 0])

    def eval(self, t):
        assert t >= 0
        assert t <= self.duration

        current_t = 0.0
        for p in self.polynomials:
            if t < current_t + p.duration:
                return p.eval(t - current_t)
            current_t = current_t + p.duration
