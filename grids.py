from abc import abstractmethod
from udacidrone.frame_utils import global_to_local,local_to_global
import numpy as np

class Grid:
    def __init__(self):
        self.grid = None
        self.edges = None

    @abstractmethod
    def create_grid(self,data, drone_altitude, safety_distance):
        None
    @abstractmethod
    def relocate_start_goal_if_necessary(self, start, goal):
        return start,goal

class StandartGrid(Grid):
    def create_grid(self, data, drone_altitude, safety_distance):
        """
        Returns a grid representation of a 2D configuration space
        based on given obstacle data, drone altitude and safety distance
        arguments.
        """

        # minimum and maximum north coordinates
        north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
        north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

        # minimum and maximum east coordinates
        east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
        east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

        # given the minimum and maximum coordinates we can
        # calculate the size of the grid.
        north_size = int(np.ceil(north_max - north_min))
        east_size = int(np.ceil(east_max - east_min))

        # Initialize an empty grid
        grid = np.zeros((north_size, east_size))

        # Populate the grid with obstacles
        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]
            if alt + d_alt + safety_distance > drone_altitude:
                obstacle = [
                    int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                    int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                    int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                    int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
                ]
                grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1
        self.grid = grid
        self.edges = grid
        self.north_min = north_min
        self.north_max = north_max
        self.east_min = east_min
        self.east_max = east_max
        self.north_offset = int(north_min)
        self.east_offset = int(east_min)
        # return grid

    def max_min_ned_to_geodetic(self, north_min, north_max, east_min, east_max, global_home):
        global_min = local_to_global([north_min, east_min, 0], global_home)
        global_max = local_to_global([north_max, east_max, 0], global_home)
        print(global_min, global_max)
        return global_min, global_max

    def sample_state_as_ned(self, global_min, global_max, global_home):
        lon = np.random.uniform(global_min[0], global_max[0])
        lat = np.random.uniform(global_min[1], global_max[1])
        geodetic_random = np.array([lon, lat, 0])
        local_ned_random = global_to_local(geodetic_random, global_home)
        return local_ned_random

    def get_random_goal(self, global_home):
        global_min, global_max = self.max_min_ned_to_geodetic(self.north_min,
                                                              self.north_max, self.east_min,
                                                              self.east_max, global_home)
        while True:
            ned_random = self.sample_state_as_ned(global_min, global_max, global_home)
            grid_goal = int(ned_random[0] - self.north_offset), int(ned_random[1] - self.east_offset)
            print("ned_random:{} grid_goal:{} grid.shape:{} ".format(ned_random, grid_goal, self.grid.shape))
            if self.grid[grid_goal[0], grid_goal[1]] == 0:
                print("found a free point")
                break
        return grid_goal

