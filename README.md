# Go Chase It - A Robotics Project

## About

A robotics project using ROS, Gazebo and OpenCV to simulate a robot that autonomously follows any ball around an environment. This is achieved using OpenCV to detect circles in from the robot's camera.

## Tools
* ROS
* C++
* Gazebo
* OpenCV

## Run
Clone and navigate to the catkin directory

````
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch

(in a new tab -> still in the catkin directory)
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch

(optional -> to view the camera output. another new tab in catkin directory)
$ source dvel/setup.bash
$ rosrun rqt_camera_view rqt_camera_view
````

_Built as part of the Robotics Software Engineering Nanodegree by Udacity_
