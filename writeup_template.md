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


*** Notes

            #print("next_node",next_node)
            #g = distance between child and current
            # F is the total cost of the node.
            # G is the distance between the current node and the start node.
            # H is the heuristic — estimated distance from the current node to the end node.            
            
            G = current_cost + graph.edges[current_node, next_node]['weight']
            H = heuristic(next_node,goal)
            F =  G + H
            cost = F

                # 𝑔 models the cost of performing actions, irrespective of the environment,
                # ℎ models the cost based on the environment, i.e., the distance to the goal.
                #  𝑐𝑛𝑒𝑤=𝑐+𝑔()+ℎ().


#BurkeA Start:
Both py files(backyard_flyer_solution.py and motion_planning.py) has almost same structre, code flow maintained by states and 3 callback functions.But motion_planning.py has one more state (PLANNING) which is not in backyard flyer. States transtions has slight diffence,  

File | States 
--- | --- 
backyard_flyer_solution.py | MANUAL>ARMING>TAKEOFF>WAYPOINT>LANDING>DISARMING>MANUAL
motion_planning.py | MANUAL>ARMING>**PLANNING**>TAKEOFF>WAYPOINT>LANDING>DISARMING>MANUAL

PLANNING state take responsibiliy of calculate_box function in backyard flyer. And local_position_callback function now has clearer responsibility.

###particularly in the plan_path() method and functions provided in planning_utils.py and describe what's going on there

plan_path:

Swithces to state PLANNING which will trigger to drone take off. Drone will wait on WAYPOINT state until function complete. Loads obstacle map. In turn, create our grid with obstacles. Using A* with eucledian heuristic function,  find a way from start to goal only using only 4 directions(LEFT,RIGHT,TOP,DOWN). Path will be pruned and finally found path will be converted to waypoint array which will be added to waypoint list use in WAYPOINT state. 


create_grid > Creates a 2D Grid with obstacles from supplied data. obstacles are creation rule is, if there is a obstacle and drone altitude is lower than this obstacle point add safety distance to four directions of obstacle, mark it is occupied.
valid_actions> List possible action within x,y plane to direct neighbours
a_star > A* algorithm 
heuristic function> Euclidean distance

#BurkeA End:



These scripts contain a basic planning implementation that includes...

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


