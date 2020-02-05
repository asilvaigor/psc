from decision_making.AStarPlanner import AStarPlanner
from decision_making.MeshNode import MeshNode


class DroneStub:
    def __init__(self, mesh_node):
        self.mesh_node = mesh_node


def remap(path, nodes):
    remap = {}
    for k in path:
        remap[k] = []
        for n in path[k]:
            for i in range(len(nodes)):
                if nodes[i] == n:
                    remap[k].append(i)
    return remap


if __name__ == "__main__":
    # Defining graph
    nodes = [MeshNode(0, 0, 0), MeshNode(0, 1, 0), MeshNode(1, 0, 0),
             MeshNode(1, 1, 0), MeshNode(2, 0, 0)]
    nodes[0].add_edge(nodes[1])
    nodes[0].add_edge(nodes[2])
    nodes[0].add_edge(nodes[3])
    nodes[1].add_edge(nodes[3])
    nodes[1].add_edge(nodes[4])
    nodes[2].add_edge(nodes[3])
    nodes[3].add_edge(nodes[4])

    # Defining initial and goal positions
    drones = {1: DroneStub(nodes[0]), 2: DroneStub(nodes[1])}
    goal_poses = {1: nodes[4], 2: nodes[2]}

    # Running algorithm
    planner = AStarPlanner()
    path = planner.plan(drones, goal_poses)
    r = remap(path, nodes)

    print(r)
    assert(r == {1: [0, 3, 4], 2: [1, 0, 2]})
