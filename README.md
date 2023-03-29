# Nonholonomic Mobile Robot Path Planning

## Introduction
This repository implements Reed-Shepp curve combine with the Dijkstra global planner in ROS. When a goal pose is setted, this algorithm will subscribe the global path, split it to several waypoints, and design a collision free Reed-Shepp curve between waypoints.  

## Requirement
ROS (only tested in noetic), Gazebo 11, numpy, math, bidict

## Installation

```
# create a ROS workspace
mkdir -p /WS_NAME/src
# go to the workspace 
cd ~/WS_NAME/src
# clone the repository
git clone https://github.com/balloon61/Nonholonomic-Mobile-Robot-Path-Planning.git
cd ..
catkin_make
```

## Reed-Shepp Curve

A list of test points are shown in this section.

## Map
Building a world in Gazebo, and using the map_server in ROS to build the Map so that it's possible to check if the path is collision free.

![Screenshot from 2023-03-26 20-32-11](https://user-images.githubusercontent.com/55338365/227834849-44c5d3f8-aa52-473b-8b85-2ed2f0b968b0.png)
![Screenshot from 2023-03-26 20-30-51](https://user-images.githubusercontent.com/55338365/227834856-3c7e17a6-2e15-470d-a1ac-2c53abae4acc.png)

## Test

A test of the full algorithm is provided in this section.

## Visualization

Publish the whole collision free Reed-Shepp curve as a path topic so that it can be viewed in Rviz. 
