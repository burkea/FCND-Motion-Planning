from abc import abstractmethod
from action import DiagonalAction,StandartAction

class ValidMove:
    def __init__(self,grid):
        self.grid = grid

    @abstractmethod
    def valid_actions(self,current_node):
        None






class DiagonalMove(ValidMove):

    def __init__(self,grid):
        super().__init__(grid)

    def valid_actions(self, current_node):
        """
        Returns a list of valid actions given a grid and current node.
        """
        valid_actions = list(DiagonalAction)
        n, m = self.grid.shape[0] - 1, self.grid.shape[1] - 1
        x, y = current_node

        # check if the node is off the grid or
        # it's an obstacle

        if x - 1 < 0 or self.grid[x - 1, y] == 1:
            valid_actions.remove(DiagonalAction.NORTH)
        if x + 1 > n or self.grid[x + 1, y] == 1:
            valid_actions.remove(DiagonalAction.SOUTH)
        if y - 1 < 0 or self.grid[x, y - 1] == 1:
            valid_actions.remove(DiagonalAction.WEST)
        if y + 1 > m or self.grid[x, y + 1] == 1:
            valid_actions.remove(DiagonalAction.EAST)

        if (x - 1 < 0 or y - 1 < 0) or self.grid[x - 1, y - 1] == 1:
            valid_actions.remove(DiagonalAction.NORTH_WEST)
        if (x - 1 < 0 or y + 1 > m) or self.grid[x - 1, y + 1] == 1:
            valid_actions.remove(DiagonalAction.NORTH_EAST)
        if (x + 1 > n or y - 1 < 0) or self.grid[x + 1, y - 1] == 1:
            valid_actions.remove(DiagonalAction.SOUTH_WEST)
        if (x + 1 > n or y + 1 > m) or self.grid[x + 1, y + 1] == 1:
            valid_actions.remove(DiagonalAction.SOUTH_EAST)

        return valid_actions

class StandartMove(ValidMove):

    def __init__(self,grid):
        super().__init__(grid)

    def valid_actions(self, current_node):
        """
        Returns a list of valid actions given a grid and current node.
        """
        valid_actions = list(StandartAction)
        n, m = self.grid.shape[0] - 1, self.grid.shape[1] - 1
        x, y = current_node

        # check if the node is off the grid or
        # it's an obstacle

        if x - 1 < 0 or self.grid[x - 1, y] == 1:
            valid_actions.remove(StandartAction.NORTH)
        if x + 1 > n or self.grid[x + 1, y] == 1:
            valid_actions.remove(StandartAction.SOUTH)
        if y - 1 < 0 or self.grid[x, y - 1] == 1:
            valid_actions.remove(StandartAction.WEST)
        if y + 1 > m or self.grid[x, y + 1] == 1:
            valid_actions.remove(StandartAction.EAST)

        return valid_actions


