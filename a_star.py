from abc import abstractmethod
from queue import PriorityQueue

class SearchAlgorithm:
    @abstractmethod
    def run(self, h, start, goal):
        None


class A_Star_Graph(SearchAlgorithm):
    def __init__(self,graph,move):
        self.graph = graph
        self.move = move

    def run(self, h, start, goal):
        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal:
                print('Found a path.')
                found = True
                break
            else:
                # print("current_node:", current_node)
                for next_node in self.graph[current_node]:
                    # print("next_node:", next_node)
                    # print("edge:", self.graph.edges[current_node, next_node])

                    # cost = self.graph.edges[current_node, next_node]['weight']
                    # # print("cost:", cost)
                    # branch_cost = current_cost + cost
                    # queue_cost = branch_cost + h(next_node, goal)

                    # F is the total cost of the node.
                    # G is the distance between the current node and the start node.
                    # H is the heuristic — estimated distance from the current node to the end node.
                    cost = self.graph.edges[current_node, next_node]['weight']
                    G = current_cost + cost
                    H = h(next_node, goal)
                    F = G + H
                    queue_cost = F

                    '''
                    for action in valid_actions(grid, current_node):
                        # get the tuple representation
                        da = action.delta
                        next_node = (current_node[0] + da[0], current_node[1] + da[1])
                        branch_cost = current_cost + action.cost
                        queue_cost = branch_cost + h(next_node, goal)
                    '''
                    if next_node not in visited:
                        visited.add(next_node)
                        branch[next_node] = (G, current_node, next_node)
                        queue.put((queue_cost, next_node))
        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append(goal)
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
        return path[::-1], path_cost


class A_Star_Grid(SearchAlgorithm):

    def __init__(self,grid,move):

        self.move = move
        self.grid = grid

    def run(self, h, start, goal):

        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal:
                print('Found a path.')
                found = True
                break
            else:
                for action in self.move.valid_actions(current_node):
                    # get the tuple representation
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)


                    if next_node not in visited:
                        visited.add(next_node)
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))

        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append(goal)
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
        return path[::-1], path_cost