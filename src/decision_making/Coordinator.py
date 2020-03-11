from decision_making.Delta import Delta
from decision_making.trajectory.Path import Path


class Coordinator:
    def __init__(self):
        self.__paths = {}

    def coordinate(self, nodes_paths):
        """
        Generates coordinated paths by adjusting drone velocities given the current path, so the
        drones won't collide. This algorithm is based on the article by  Robert Ghrist, Jason
        M. O'Kane and Steven M. LaValle, untitled Computing Pareto Optimal Coordinations on
        Roadmaps, published on The International Journal of Robotics Research, year 2005.
        :param nodes_paths: List of MeshNode objects representing a path to follow.
        :return: Dict of Path objects, a coordinated roadmap for the drones.
        """
        delta = Delta(nodes_paths)
        for drone_id in nodes_paths:
            self.__paths[drone_id] = Path()

        for drone_id in nodes_paths:
            self.__paths[drone_id] = Path(nodes_paths[drone_id])
        return self.__paths
