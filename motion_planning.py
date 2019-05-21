import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

# from planning_utils import a_star, heuristic, create_grid
from planning_utils import heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local,local_to_global
from a_star import A_Star_Graph,A_Star_Grid
from moves import DiagonalMove,StandartMove
from random_location_on_grid import RandomLocationOnGrid
from bootstrap import BootStrap

from grids import StandartGrid

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()



def show_plan(grid,grid_start,grid_goal,path):
    import matplotlib.pyplot as plt
    # plt.rcParams['figure.figsize'] = 14, 14

    fig = plt.figure()

    # Show obstacles
    plt.imshow(grid, cmap='Greys', origin='lower')
    # show start and goal positions
    plt.plot(grid_start[1], grid_start[0], 'o')
    plt.plot(grid_goal[1], grid_goal[0], 'x')
    # show path
    if path is not None:
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')

    plt.xlabel('NORTH')
    plt.ylabel('EAST')

    plt.show()

    print("---Plot graph finish---")
    plt.show(block=False)

class MotionPlanning(Drone):

    def __init__(self, connection,boot_strap):
        super().__init__(connection)
        self.boot_strap = boot_strap
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        self.local_velocity
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)




    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE : TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            line = f.readline() # lat0 37.792480, lon0 -122.397450

        latlon = line.strip().split(',')
        lat0 = float(latlon[0].lstrip("lat0 "))
        lon0 = float(latlon[1].lstrip("lon0 "))

        print("lon0:{} :lat0{}".format(lon0,lat0))
        # DONE : TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0,lat0,0)
        
        # DONE:  TODO: retrieve current global position

        geodetic_current_coordinates = [self._longitude,self._latitude,self._altitude]
        print("geodetic_current_coordinates:",geodetic_current_coordinates)
        geodetic_home_coordinates = self.global_home
        print("self.global_home:",self.global_home)
        # DONE: - TODO: convert to current local position using global_to_local()
        current_local_pos_ned = global_to_local(geodetic_current_coordinates,geodetic_home_coordinates)
        print("current_local_pos_ned:",current_local_pos_ned)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset,(north_min,north_max,east_min,east_max) = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        standart_grid = StandartGrid()
        standart_grid.create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print("North offset = {0}, east offset = {1}".format(standart_grid.north_offset, standart_grid.east_offset))

        # Define starting point on the grid (this is just grid center)
        grid_start = (-standart_grid.north_offset, -standart_grid.east_offset)
        # DONE : TODO: convert start position to current position rather than map center - DONE
        grid_start = (int(current_local_pos_ned[0] - standart_grid.north_offset), int(current_local_pos_ned[1] - standart_grid.east_offset))

        # Set goal as some arbitrary position on the grid
        grid_goal = (-standart_grid.north_offset + 10, -standart_grid.east_offset + 10)
        # DONE : TODO: adapt to set goal as latitude / longitude position and convert
        # r = RandomLocationOnGrid(self.global_home)
        # grid_goal = r.get_random_goal(north_min, north_max, east_min, east_max,north_offset,east_offset, grid)
        grid_goal = standart_grid.get_random_goal(self.global_home)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)

        a_star = A_Star_Grid(standart_grid.grid, self.boot_strap.create("MOVE_ABILITY")(standart_grid.grid))

        path, _ = a_star.run(heuristic, grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("A* finished.")


        #Show Path
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        prune = boot_strap.create("PRUNING_ALGORITHM")(standart_grid.grid)
        path = prune.run(path)

        # Convert path to waypoints
        waypoints = [[p[0] + standart_grid.north_offset, p[1] + standart_grid.east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        #self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        #self.stop_log()


if __name__ == "__main__":
    boot_strap = BootStrap()
    boot_strap.add("MOVE_ABILITY", "moves.DiagonalMove") #DiagonalMove,StandartMove
    boot_strap.add("PRUNING_ALGORITHM", "pruning.Bresenham")  # Bresenham,Collinearity

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn,boot_strap)
    time.sleep(1)

    drone.start()
