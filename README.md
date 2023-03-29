# Nonholonomic Mobile Robot Path Planning

## Introduction
This repository implements Reed-Shepp curve combined with the Dijkstra global planner in ROS. When a goal pose is set, this algorithm will subscribe to the global path, split it into several waypoints, and design a collision-free Reed-Shepp curve between waypoints.

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
A list of the Reed-Shepp function testing results are shown in this section, the result of each test point is sorted by the total length of the Reed-Shepp Curve.

```
[INFO] [1680115919.265895]: goal pose (0.5, 0.5, 1.5)
Direction: Forward, Steering: Left, Length: 0.5468855919024702
Direction: Backward, Steering: Right, Length: 0.330041103168711
Direction: Forward, Steering: Left, Length: 0.6230733049288188

Direction: Backward, Steering: Right, Length: 0.2181353779272155
Direction: Forward, Steering: Left, Length: 1.0879075480222795
Direction: Backward, Steering: Right, Length: 0.193957074050505

Direction: Forward, Steering: Left, Length: 0.6230733049288184
Direction: Backward, Steering: Right, Length: 0.330041103168711
Direction: Backward, Steering: Left, Length: 0.5468855919024707

Direction: Backward, Steering: Right, Length: 0.19395707405050455
Direction: Forward, Steering: Left, Length: 1.0879075480222795
Direction: Forward, Steering: Right, Length: 0.21813537792721593

Direction: Backward, Steering: Right, Length: 0.7296133513678151
Direction: Backward, Steering: Left, Length: 0.42288213756719756
Direction: Forward, Steering: Right, Length: 0.42288213756719756
Direction: Forward, Steering: Left, Length: 1.6161509237665799

Direction: Forward, Steering: Left, Length: 1.6562143373984497
Direction: Forward, Steering: Right, Length: 0.4495502811200134
Direction: Backward, Steering: Left, Length: 0.4495502811200134
Direction: Backward, Steering: Right, Length: 0.7428862248415768

Direction: Backward, Steering: Right, Length: 0.4769416187441671
Direction: Backward, Steering: Straight, Length: 1.4658807421549507
Direction: Backward, Steering: Left, Length: 1.5707963267948966
Direction: Forward, Steering: Right, Length: 0.5477379455390636

[INFO] [1680115919.268779]: goal pose (0.5, 0.5, 0.5)
Direction: Backward, Steering: Right, Length: 0.27176064354353047
Direction: Forward, Steering: Left, Length: 0.5886963580423875
Direction: Backward, Steering: Right, Length: 0.3604570015859183

Direction: Forward, Steering: Left, Length: 1.0141676297453435
Direction: Forward, Steering: Right, Length: 0.3961143979764107
Direction: Backward, Steering: Left, Length: 0.3961143979764107
Direction: Backward, Steering: Right, Length: 0.2780611662074781

Direction: Backward, Steering: Left, Length: 0.9216831390846276
Direction: Forward, Steering: Right, Length: 0.18935410885979906
Direction: Forward, Steering: Left, Length: 1.6110372479444264

Direction: Forward, Steering: Left, Length: 2.030555405645366
Direction: Backward, Steering: Right, Length: 0.18935410885979906
Direction: Backward, Steering: Left, Length: 1.719909514505165

[INFO] [1680115919.272154]: goal pose (0.5, -0.5, 1.5)
Direction: Forward, Steering: Right, Length: 0.0933575602527773
Direction: Forward, Steering: Left, Length: 0.8102989147330824
Direction: Backward, Steering: Right, Length: 0.8102989147330824
Direction: Backward, Steering: Left, Length: 0.027240269213387514

Direction: Backward, Steering: Right, Length: 0.8207959969305758
Direction: Forward, Steering: Left, Length: 0.8000716947612508
Direction: Forward, Steering: Right, Length: 0.12086769169182698

Direction: Forward, Steering: Left, Length: 0.8478292989646383
Direction: Backward, Steering: Right, Length: 0.776010903147784
Direction: Forward, Steering: Left, Length: 0.1238402021124223

Direction: Backward, Steering: Right, Length: 1.5272402692133875
Direction: Backward, Steering: Left, Length: 0.8102989147330824
Direction: Forward, Steering: Right, Length: 0.8102989147330824
Direction: Forward, Steering: Left, Length: 1.5933575602527768
```

## Map
Building a world in Gazebo, and using the map_server in ROS to build the Map so that it's possible to check if the path is collision free.

![Screenshot from 2023-03-26 20-32-11](https://user-images.githubusercontent.com/55338365/227834849-44c5d3f8-aa52-473b-8b85-2ed2f0b968b0.png)
![Screenshot from 2023-03-26 20-30-51](https://user-images.githubusercontent.com/55338365/227834856-3c7e17a6-2e15-470d-a1ac-2c53abae4acc.png)

## Test

A test of the full algorithm is provided in this section.

## Visualization

Publish the whole collision free Reed-Shepp curve as a path topic so that it can be viewed in Rviz. 
