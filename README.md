# Where Am I - A Robotics Project

## About

A robotics project using ROS, Gazebo, C++ and Monte Carlo Localization to determine a robot's location inside a simulated world. The particles are visualized on a 2D map of the 3D simulated world. 

## Tools
* ROS
* C++
* Gazebo

## Run
Clone and navigate to the catkin directory

````
$ cd catkin
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch

(in a new terminal)
$ cd catkin
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch

(option: control robot with keyboard)
(in a new terminal)
$ cd catkin
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

````

_Built as part of the Robotics Software Engineering Nanodegree by Udacity_
