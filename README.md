## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

* Both py files(backyard_flyer_solution.py and motion_planning.py) has almost same structre, 
code flow maintained by states and 3 callback functions.But motion_planning.py has one more state 
(PLANNING) which is not in backyard flyer. States transtions has slight diffence,  

File | States 
--- | --- 
backyard_flyer_solution.py | MANUAL>ARMING>TAKEOFF>WAYPOINT>LANDING>DISARMING>MANUAL
motion_planning.py | MANUAL>ARMING>**PLANNING**>TAKEOFF>WAYPOINT>LANDING>DISARMING>MANUAL

PLANNING state take responsibiliy of calculate_box function in backyard flyer. As a result,
 local_position_callback function now has clearer responsibility. In States enum used auto() to automatically uniquely enumerate states.


###particularly in the plan_path() method and functions provided in planning_utils.py and describe what's going on there

plan_path:

Swithces to state PLANNING which will trigger to drone take off. Drone will wait on WAYPOINT 
state until function complete. Loads obstacle map. In turn, create our grid with obstacles. 
Using A* with eucledian heuristic function,  find a way from start to goal only using only 
4 directions(LEFT,RIGHT,TOP,DOWN). Path will be pruned and finally found path will be converted 
to waypoint array which will be added to waypoint list which will use in WAYPOINT state. 


create_grid > Creates a 2D Grid with obstacles from supplied data. obstacles are creation rule is, 
if there is a obstacle and drone altitude is lower than this obstacle point add safety distance to 
four directions of obstacle, mark it is occupied.
valid_actions> List possible action within x,y plane to direct neighbours
a_star > A* algorithm 
heuristic function> Euclidean distance


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position - Done
collider.csv file is opened via <b>open()</b> method. And first line is read which are initial position of drone. Line is splitted by comma and extract values with <b>rstrip</b> method. Finally converted to float. In turn, <b>self.set_home_position</b> method is called.

#### 2. Set your current local position - Done
To get our current coordinates or local position relative to global home in NED Frame, used global_to_local() function. Our current geodetic coordinates can be taken from self._longitude,self._latitude,self._altitude variables. And our geodetic home can be taken from self.global_home variable.

#### 3. Set grid start position from local position
north_offset and east_offset are the vector to (0,0) of the grid.  This
vector is subtracted from the the current local position in the map
reference frame to get the start position in the grid reference frame.


### Custom Implementations...
I have added some classes to inject and test behaviour of different algorithms. Probably better DI implementation are available on pip. but  I've wrote a simple BootStrap class to store and create injected classes. Other important aspect is, I've used abstraction more than required to simplfy code usage, since some places violates Interface Segregation Principle.  
* moves.py
<b>ValidMove :</b> Generates valid moved for given grid and location
<b>StandartMove(ValidMove) : </b>Generates 4 direction. replacement of valid_actions in planning_utils.py
<b>DiagonalMove((ValidMove) :</b>Generates 8 direction including diagonal movements.
*pruning.py 
<b>Prune :</b> Base class for pruning operations
<b>Collinearity(Prune) :</b> Collinearity implementation
<b>Bresenham(Prune) :</b> Bresenham ray Tracing method impl.
* action.py
<b>StandartAction :</b> renamed version of Action enum in planning_util.py
<b>DiagonalAction :</b> diagonal action and respective costs
* grids.py
<b>Grid :</b> Base of all grids. I've abstracted available navigable space generation job to this class. Grids can sample a random point which not collide with obstacles. Also the provide relocate_start_goal_if_necessary() method to any point not in edge or skeleton, can move to valid location which in turn search algorithm could find a path. All classes has grid and edges member variables. grid is used for collision detection and edges is used for search algoritm
<b>StandartGrid(Grid) :</b> Standart grid with obstacles are 1 in grid.
<b>MedialAxis(Grid):</b> MedialAxis implementation. The code used
was essentially the code presented in lectures. Specifically Lesson 3, section
13 'Medial Axis Exercise'.   This method plans considerably faster than standart grid method.
<b>VoronoiGrid(Grid):</b>VoronoiGrid implementation. The code used
was essentially the code presented in lectures. Specifically Lesson 3, section
14 'Voronoi Graph Exercise'.   

* a_star.py 
<b>SearchAlgorithm :</b> abstraction for A*(for now)
<b>A_Star_Graph(SearchAlgorithm):</b> Graph based implementation of A*. It used only in VoronoiGrid
<b>A_Star_Grid(SearchAlgorithm):</b> Grid based implementation of A*. It can be used in StandartGrid and MedialAxis

We can inject any of these classes in main() method.
<code>
boot_strap.add("MOVE_ABILITY", "moves.DiagonalMove") #DiagonalMove,StandartMove
boot_strap.add("PRUNING_ALGORITHM", "pruning.Collinearity")  # Bresenham,Collinearity
boot_strap.add("SEARCH_ALGORITHM", "a_star.A_Star_Grid")  # A_Star_Graph,A_Star_Grid
boot_strap.add("GRID", "grids.StandartGrid")  # StandartGrid,MedialAxis,VoronoiGrid
</code>
Ability to inject does not mean they all working together. :( For example VoronoiGrid is used with only A_Star_Graph and MOVE_ABILITY is not any impact on A* as expected. Checking classes is future work at this moment.  

Available compositions as follows:

Search | Grid | Prune | Move | Action | Usable
---|---|---|---|---|---
A_Star_Grid | StandartGrid | Bresenham  | StandartMove | StandartAction
A_Star_Grid | StandartGrid | Bresenham  | StandartMove | DiogonalAction
A_Star_Grid | StandartGrid | Bresenham  | DiagonalMove | StandartAction
A_Star_Grid | StandartGrid | Bresenham  | DiagonalMove | DiogonalAction
A_Star_Grid | StandartGrid | Collinearity  | StandartMove | StandartAction
A_Star_Grid | StandartGrid | Collinearity  | StandartMove | DiogonalAction
A_Star_Grid | StandartGrid | Collinearity  | DiagonalMove | StandartAction
A_Star_Grid | StandartGrid | Collinearity  | DiagonalMove | DiogonalAction
A_Star_Grid | MedialAxis | Bresenham  | DiagonalMove | DiogonalAction |
A_Star_Grid | MedialAxis | Collinearity  | DiagonalMove | DiogonalAction
A_Star_Grid | A_Star_Graph | Bresenham  | DiagonalMove | DiogonalAction
A_Star_Grid | A_Star_Graph | Collinearity  | DiagonalMove | DiogonalAction

#### 4. Set grid goal position from geodetic coords
Can be set via drone.CUSTOM_START_LON, drone.CUSTOM_START_LAT. Otherwise grid implementation generates a random point. 

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
DiagonalMove class is implemented by adding the actions
NE, NW, SE, and SW with cost of sqrt(2).  valid_actions() is also modified to
remove NE, NW, SE, or SW when it would move off the grid, or collide.

#### 6. Cull waypoints 
All pruning operation encapsulated in  Collinearity(Prune) and Bresenham(Prune) classes which are presented in Lesson 3, Section 5 and 8.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

Deadband: Deadband is implemented in calculate_deadband() method. and used local_position_callback() function. It is very simple apporach. Deadband is considered a function like f(radius) = a * ||velocity|| + b where a is constant which is controlling how big is radius. a is a constant for minimum radius.

[//]: # (Bürke Atilla // May 2019 )
