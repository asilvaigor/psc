import heapq


class AStarPlanner:
    def __init__(self):
        pass

    def plan(self, start_node, goal_node):
        """
        Calculates trajectory using A* algorithm. Worst-case O(n + e log(e))
        :param start_node: Beginning node.
        :param goal_node: Goal node.
        :return: List of nodes representing the shortest path.
        """
        if start_node == goal_node:
            return []

        # Data structures used
        pq = []  # Heap, with triplets (heuristic, cost, drone)
        visited = {}
        came_from = {}

        # First node
        cost = 0  # Cost in graph
        dist = goal_node.dist(start_node)  # Heuristic compensation
        heuristic = cost + dist
        pq.append((heuristic, cost, start_node))
        visited[start_node] = True

        # Running algorithm
        while len(pq) > 0:
            # Get closest node
            cur = heapq.heappop(pq)
            if cur[2] == goal_node:
                break

            # Insert edges in pq
            for n in cur[2].edges:
                if n not in visited:
                    cost = cur[1] + cur[2].dist(n)
                    dist = goal_node.dist(n)
                    heuristic = cost + dist
                    heapq.heappush(pq, (heuristic, cost, n))
                    visited[n] = True
                    came_from[n] = cur[2]

        return self.__reconstruct_path(came_from, start_node, goal_node)

    @staticmethod
    def __reconstruct_path(came_from, start_node, goal_node):
        """
        Reconstructs a list of nodes representing a path to follow from a dict of nodes
        representing where the path came from.
        :param came_from: Dict of Node containing previous node in path.
        :param start_node: Node that is the beginning.
        :param goal_node: Node that is the end.
        :return: Array of Node containing a path to follow.
        """
        path = [goal_node]
        cur = goal_node
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)

        if path[-1] != start_node:
            return []
        return path[::-1]
