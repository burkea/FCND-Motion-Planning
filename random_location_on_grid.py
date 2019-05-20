from udacidrone.frame_utils import global_to_local,local_to_global
import numpy as np

class RandomLocationOnGrid:
    def __init__(self,global_home):
        self.global_home = global_home
        # self.north_min = north_min
        # self.north_max = north_max
        # self.east_min = east_min
        # self.east_max = east_max

    def max_min_ned_to_geodetic(self,north_min, north_max, east_min, east_max):
        global_min = local_to_global([north_min, east_min, 0], self.global_home)
        global_max = local_to_global([north_max, east_max, 0], self.global_home)
        print(global_min, global_max)
        return global_min, global_max

    def sample_state_as_ned(self,global_min, global_max, global_home):
        lon = np.random.uniform(global_min[0], global_max[0])
        lat = np.random.uniform(global_min[1], global_max[1])
        geodetic_random = np.array([lon, lat, 0])
        local_ned_random = global_to_local(geodetic_random, global_home)
        return local_ned_random

    def get_random_goal(self,north_min, north_max, east_min, east_max,north_offset,east_offset,grid):
        global_min, global_max = self.max_min_ned_to_geodetic(north_min, north_max, east_min, east_max)
        while True:
            ned_random = self.sample_state_as_ned(global_min, global_max, self.global_home)
            grid_goal = int(ned_random[0] - north_offset), int(ned_random[1] - east_offset)
            print("ned_random:{} grid_goal:{} grid.shape:{} ".format(ned_random, grid_goal, grid.shape))
            if grid[grid_goal[0], grid_goal[1]] == 0:
                print("found a free point")
                break
        return grid_goal