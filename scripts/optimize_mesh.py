from decision_making.UniformMesh import UniformMesh
from decision_making.CGALMesh import CGALMesh
from representations.obstacles.ObstacleCollection import ObstacleCollection
from decision_making.AStarPlanner import AStarPlanner
from decision_making.trajectory.Path import Path
from decision_making.MeshNode import MeshNode

from scipy.optimize import minimize

import random
import numpy as np

TRIES = 400


def objective_function(x):

    def calc_path_len(path):
        dist = 0.0
        for i in range(1, len(path)):
            dist += path[i].dist(path[i-1])
        return dist
    mesh = CGALMesh(x[0], x[1], x[2], x[3], x[4])

    mesh.discretize(ObstacleCollection(), {}, {})

    nb_nodes = len(mesh.nodes)

    planner = AStarPlanner()

    tot = 0.0

    for i in range(TRIES):
        init_node = mesh.nodes[random.randint(0, nb_nodes-1)]
        end_node = mesh.nodes[random.randint(0, nb_nodes-1)]
        path = planner.plan(init_node, end_node)
        path_len = calc_path_len(path)
        diff = abs(path_len - end_node.dist(init_node))
        tot += diff

    return tot/TRIES


x0 = np.array([30, 0.2, 0.02, 2, 0.3])

res = minimize(objective_function, x0, method='nelder-mead',
               options={'xatol': 0.1, 'maxiter': 100, 'disp': True})
print(res)
