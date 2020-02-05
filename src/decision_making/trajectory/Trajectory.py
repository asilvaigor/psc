import numpy as np

from Polynomial4D import Polynomial4D
from decision_making.MeshNode import MeshNode
from representations.Constants import POLYNOMIAL_INT
from representations.Constants import POLYNOMIAL_SIZE

# Trajectory that composed by an array with polynomials and their respective durations
class Trajectory:
    def __init__(self):
        self.polynomials = None
        self.duration = None

    def create_trajectory(self, path):
        i = 0
        while i < len(path):
            if ((len(path) - POLYNOMIAL_INT - i) % (POLYNOMIAL_SIZE - POLYNOMIAL_INT)) == 0:
                pol_size = POLYNOMIAL_SIZE
            else:
                pol_size = POLYNOMIAL_SIZE + 1

            #x = find
            y = path[i:i + pol_size].x
            px = np.polyfit(x, y, pol_size)

            y = path[i:i + pol_size].y
            py = np.polyfit(x, y, pol_size)

            y = path[i:i + pol_size].z
            pz = np.polyfit(x, y, pol_size)

            y = [0] * pol_size
            pyaw = np.polyfit(x, y, 0)

            duration = x[pol_size - 1]

            self.polynomials.append(Polynomial4D(duration, px, py, pz, pyaw))

            i = i + pol_size - POLYNOMIAL_INT


    def find_times(self):
        # TODO
        pass

    def eval(self, t):
        assert t >= 0
        assert t <= self.duration

        current_t = 0.0
        for p in self.polynomials:
            if t < current_t + p.duration:
                return p.eval(t - current_t)
            current_t = current_t + p.duration
