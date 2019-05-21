from abc import abstractmethod
import numpy as np
from bresenham import bresenham


class Prune:

    def __init__(self,grid):
        self.grid = grid
    @abstractmethod
    def run(self,path):
        None

    def point(self,p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)



class Collinearity(Prune):

    def collinearity_check(self,p1, p2, p3, epsilon=1e-6):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def run(self,path):
        pruned_path = [p for p in path]
        # TODO: prune the path!

        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i + 1])
            p3 = self.point(pruned_path[i + 2])

            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if self.collinearity_check(p1, p2, p3):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i + 1])
            else:
                i += 1
        return pruned_path




class Bresenham(Prune):
    def __init__(self,grid):
        super().__init__(grid)

    def run(self,path):
        pruned_path = [p for p in path]
        i = 0
        while i < len(pruned_path) - 2:
            p1 = pruned_path[i]
            p2 = pruned_path[i + 1]
            p3 = pruned_path[i + 2]
            cells = list(bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1])))
            hit = False
            # Collision check
            for c in cells:
                # First check if we're off the map
                if np.amin(c) < 0 or c[0] >= self.grid.shape[0] or c[1] >= self.grid.shape[1]:
                    hit = True
                    break
                # Next check if we're in collision
                if self.grid[c[0], c[1]] == 1:
                    hit = True
                    break
            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if not hit:
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i + 1])
            else:
                i += 1
        return pruned_path