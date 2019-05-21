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

from skimage.morphology import medial_axis
from skimage.util import invert

class MedialAxis(StandartGrid):
    def __init__(self):
        self.skeleton = None
    def create_grid(self,data, drone_altitude, safety_distance):
        super().create_grid(data, drone_altitude, safety_distance)
        self.skeleton = medial_axis(invert(self.grid))
        self.grid = self.grid
        self.edges = invert(self.skeleton).astype(np.int)

    def relocate_start_goal_if_necessary(self, start, goal):
        skel_cells = np.transpose(self.skeleton.nonzero())
        start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
        near_start = skel_cells[start_min_dist]
        goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
        near_goal = skel_cells[goal_min_dist]
        return tuple(near_start), tuple(near_goal)


from scipy.spatial import Voronoi, voronoi_plot_2d
import networkx as nx
import numpy.linalg as LA
from bresenham import bresenham


class VoronoiGrid(StandartGrid):
    def create_grid(self, data, drone_altitude, safety_distance):
        """
        Returns a grid representation of a 2D configuration space
        along with Voronoi graph edges given obstacle data and the
        drone's altitude.
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
        # Initialize an empty list for Voronoi points
        points = []
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
                # add center of obstacles to points list
                points.append([north - north_min, east - east_min])

        # TODO: create a voronoi graph based on
        # location of obstacle centres
        graph = Voronoi(points)

        # TODO: check each edge from graph.ridge_vertices for collision
        edges = []
        for v in graph.ridge_vertices:
            p1 = graph.vertices[v[0]]
            p2 = graph.vertices[v[1]]
            cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
            hit = False

            for c in cells:
                # First check if we're off the map
                if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                    hit = True
                    break
                # Next check if we're in collision
                if grid[c[0], c[1]] == 1:
                    hit = True
                    break

            # If the edge does not hit on obstacle
            # add it to the list
            if not hit:
                # array to tuple for future graph creation step)
                p1 = (p1[0], p1[1])
                p2 = (p2[0], p2[1])
                edges.append((p1, p2))

        self.north_min = north_min
        self.north_max = north_max
        self.east_min = east_min
        self.east_max = east_max
        self.north_offset = int(north_min)
        self.east_offset = int(east_min)

        self.grid = grid
        self.underlying_edges = edges

        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            # print(">",p1,p2)
            p1 = tuple(map(int, p1))
            p2 = tuple(map(int, p2))
            # print(">>",p1,p2)
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)

        self.grid = grid
        self.edges = G

    def closest_point(self, graph, current_point):
        # iterate over graph
        dist = 9999999
        closest_point = None
        for p in graph.nodes:
            d = LA.norm(np.array(p) - np.array(current_point))
            if d < dist:
                dist = d
                closest_point = p
        return closest_point

    def relocate_start_goal_if_necessary(self, start, goal):
        start_ne_g = self.closest_point(self.edges, start)
        goal_ne_g = goal  # self.closest_point(self.grid, goal)
        return start_ne_g, goal_ne_g

    def get_random_goal(self, global_home):
        global_min, global_max = self.max_min_ned_to_geodetic(self.north_min,
                                                              self.north_max, self.east_min,
                                                              self.east_max, global_home)
        # while True:
        ned_random = self.sample_state_as_ned(global_min, global_max, global_home)
        grid_goal = int(ned_random[0] - self.north_offset), int(ned_random[1] - self.east_offset)
        print("ned_random:{} grid_goal:{} ".format(ned_random, grid_goal))
        grid_goal = self.closest_point(self.edges, grid_goal)
        # if self.grid[grid_goal[0], grid_goal[1]] == 0:
        #    print("found a free point")
        #    break
        return grid_goal
