# debi_nav
This package is made for Turtlebot3 waffle_pi navigation in an unknown environment containing both solid and visual obstacles. The package mainly relies on an A* path planning implementation in addition to a low level model predicitve controller for goal tracking. Moreover, the package integrates sensory information by superimposing the traditional map publihsed by gmapping method with another custom map formed by processing raw camera feed in order to create a unified global map. The unified map accounts for the existence of both solid obstacles and visual obstacles with specific RGB features. Accordingly, the robot is able to avoid areas with specific colors and maneuver between walls simultaneously.

## System Requirements and Installations
- ROS Noetic
- Python3
- [Turtlebot3 simulation package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

## Dependencies
- [Casadi library](https://web.casadi.org/get/)
- [scipy library](https://pypi.org/project/scipy/)
- [pathfinding library](https://github.com/brean/python-pathfinding)
- [OpenCV](https://pypi.org/project/opencv-python/)

## Demo Terminal Commands
### First terminal
> export TURTLEBOT3_MODEL=waffle_pi
> roslaunch debi_nav project.launch
## Second terminal
> export TURTLEBOT3_MODEL=waffle_pi
> roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
## Third terminal
> roslaunch debi_nav debi.launch
