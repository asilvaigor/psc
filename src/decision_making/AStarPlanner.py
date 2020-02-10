import heapq


class AStarPlanner:
    def __init__(self):
        pass

    def plan(self, drone_nodes, goal_nodes):
        """
        Calculates trajectory using A* algorithm. Worst-case O(n + e log(e))
        :param drone_nodes: Dict of MeshNodes for each drone.
        :param goal_nodes: Dict of goal MeshNodes for each drone_id.
        :return: Dict of paths for each drone_id. A path here is a list of MeshNodes in the world
        mesh.
        """
        paths = {}

        for drone_id in drone_nodes:
            # Dealing trivial case
            if goal_nodes[drone_id] == drone_nodes[drone_id]:
                paths[drone_id] = []
                continue

            # Data structures used
            pq = []  # Heap, with triplets (heuristic, cost, drone)
            visited = {}
            came_from = {}

            # First node
            cost = 0  # Cost in graph
            dist = goal_nodes[drone_id].dist(drone_nodes[drone_id])  # Heuristic compensation
            heuristic = cost + dist
            pq.append((heuristic, cost, drone_nodes[drone_id]))
            visited[drone_nodes[drone_id]] = True

            # Running algorithm
            while len(pq) > 0:
                # Get closest node
                cur = heapq.heappop(pq)
                if cur[2] == goal_nodes[drone_id]:
                    break

                # Insert edges in pq
                for n in cur[2].edges:
                    if n not in visited:
                        cost = cur[1] + cur[2].dist(n)
                        dist = goal_nodes[drone_id].dist(n)
                        heuristic = cost + dist
                        heapq.heappush(pq, (heuristic, cost, n))
                        visited[n] = True
                        came_from[n] = cur[2]

            paths[drone_id] = self.__reconstruct_path(came_from, drone_nodes[drone_id],
                                                      goal_nodes[drone_id])

        return paths

    @staticmethod
    def __reconstruct_path(came_from, start_node, goal_node):
        """
        Reconstructs a list of nodes representing a path to follow from a dict of nodes
        representing where the path came from.
        :param came_from: Dict of MeshNode containing previous node in path.
        :param start_node: MeshNode that is the beginning.
        :param goal_node: MeshNode that is the end.
        :return: Array of nodes containing a path to follow.
        """
        path = [goal_node]
        cur = goal_node
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)

        if path[-1] != start_node:
            return []
        return path[::-1]
