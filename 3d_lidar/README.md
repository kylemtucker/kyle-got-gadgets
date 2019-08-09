## README by Kyle Tucker. 3D Lidar Scipts and usage for YDLIDAR x4 in conjunction with HEBI X8-9

# Description
Series of python scripts used for facilitating the usage of YDLIDAR x4 and Hebi X8-9 together in a rotational lidar system.
The system in question contains a Hebi rotational actuator attached to a lidar device such that the lidar is rotate around the x axis. (Where positive x is the direction the lidar faces, and the z-axis is towards the sky!)

## Dependencies:
1. Rospy, bc it is ros and in python
2. ydlidar wrapper, as well as Hebi wrapper, or hebi-py python package
3. Time, math, numpy

## Hebi Controller:
Script capable of rotating hebi actuator at a constant rate between two angle boundaries. Upon detection of an angle boundary via hebi feedback controller, velocity of rotation is reversed.
Furutre iterations will have the hebi controller pause at discrete angle locations, such that a laserscan is not being conducted while the hebi controller is moving.
This controller also outputs the current angular position of the habi actuator for future use.

## Lidar Controller:
Scipt capable of reading laserscan data, and writing valid laserscan data to a csv file after processing.
Calls the hebi controller python script to hold onto a HebiForLidar object, which contains hebi angular feedback for further data processing.
Future iterations of this may publish a direct tf between the laser-scan frame and the angle of the lidar

## Lidar to Points:
Helper functions to assist with convering lidar ranges to points through appropriate vector 3d rotational transformations. Also runs analysis on csv, plotting each laserscan as a point on a matplotlib graph.

## Log dot CSV:
CSV file holsing onto all data information. Currently saves rospy current time, current hebi angle of rotation, and outputted laserscan at this time onto one singular line. These lines can be processed further once saved.

