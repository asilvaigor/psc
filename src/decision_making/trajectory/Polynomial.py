# Class to represent a polynomial
class Polynomial:

    def __init__(self, p):
        self.p = p

    # evaluate a polynomial using horner's rule
    def eval(self, t):
        assert t >= 0
        x = 0.0
        for i in range(0, len(self.p)):
          x = x * t + self.p[len(self.p) - 1 - i]
        return x

    # compute and return derivative
    def derivative(self):
        return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])