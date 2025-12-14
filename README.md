# ROS2 Maze Solver
## Georgia Tech ME7785 Final Project

The goal of this project was to create a software package with ROS2 that enables a Turtlebot3 to solve a simple maze. The approach my partner and I took was to have the robot navigate between waypoints, that were set distances of 3' apart and then use a KNN classifier to determine what direction to turn when a wall obstructs the path of motion. Navigation between waypoints and headings was handled with ROS2 NAV2. Waypoints were processed as local waypoints and were transformed before published to NAV2 to improved readibility. Nodes in this package can be found in the '''sv_maze_nav''' directory.

The final software package was able to solve the maze in just under five minutes and completed runs 5/6 times without errors.  Out approach was limited by the standardized waypoint method we used, which was overly optimistic and only worked well in a perfect environment.  Were we to complete this project again, we would very likely use a different approach for handling waypoint target generation, but time prevented us from implementing a more robust solution.

## Credit
- Aditya Rao
- Sean Wilson
