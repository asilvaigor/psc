from __future__ import with_statement
from decision_making.UniformMesh import UniformMesh
from decision_making.CGALMesh import CGALMesh
from representations.obstacles.ObstacleCollection import ObstacleCollection
from decision_making.AStarPlanner import AStarPlanner
from decision_making.trajectory.Path import Path
from decision_making.MeshNode import MeshNode

from scipy.optimize import minimize
from scipy.optimize import differential_evolution

import random
import numpy as np
import signal

TRIES = 400


def objective_function(x):
    def objective_function_inner(x):

        print("Function call")

        def calc_path_len(path):
            dist = 0.0
            for i in range(1, len(path)):
                dist += path[i].dist(path[i-1])
            return dist
        mesh = CGALMesh(x[0], x[1], x[2], x[3], x[4])
        # mesh = UniformMesh(0.2)

        mesh.discretize(ObstacleCollection(), {}, {})

        nb_nodes = len(mesh.nodes)

        planner = AStarPlanner()

        tot = 0.0

        for i in range(TRIES):
            n1 = random.randint(0, nb_nodes-1)
            n2 = random.randint(0, nb_nodes-1)
            if n1 == n2:
                n2 = (n2+1) % nb_nodes
            init_node = mesh.nodes[n1]
            end_node = mesh.nodes[n2]
            path = planner.plan(init_node, end_node)
            path_len = calc_path_len(path)
            diff = abs(path_len/end_node.dist(init_node))
            tot += diff

        return tot/TRIES

    import signal, time
    from contextlib import contextmanager

    class TimeoutException(Exception):
        pass

    @contextmanager
    def time_limit(seconds):
        def signal_handler(signum, frame):
            raise TimeoutException, "Timed out!"

        signal.signal(signal.SIGALRM, signal_handler)
        signal.alarm(seconds)
        try:
            yield
        finally:
            signal.alarm(0)

    try:
        with time_limit(3):
            return objective_function_inner(x)
    except TimeoutException, msg:
        return 10;

x0 = np.array([30, 0.2, 0.02, 2, 0.3])

res = minimize(objective_function, x0, method='nelder-mead',
               options={'xatol': 0.1, 'maxiter': 1000, 'disp': True})
print(res)

# print(objective_function(x0))


# bounds = [(10, 80), (0.1, 1), (0.01, 0.1), (1, 3), (0.1, 1)]
#
# result = differential_evolution(objective_function, bounds, maxiter=3)
# print(result)
