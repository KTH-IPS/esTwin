# sesTwin
## Seamless Establishment and Synchronisation of Digital Twins for Multi-robot Manufacturing Systems

Package name: dtir_solution_manager

Tested on: ROS2 Humble, Ubuntu 22.04

Multi-robot manager for controlling multiple ABB, UR, and KUKA industrial robots using ROS 2.

This repository is inspired and based on kuka_drivers (https://github.com/kroshu/kuka_drivers).

The drivers of the industrial robot use in this work are:

*ABB: https://github.com/PickNikRobotics/abb_ros2

*KUKA: https://github.com/kroshu/kuka_drivers

*UR: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver


Usage:
1. $ ros2 launch solution.launch.py
2. $ ros2 node run dtir_solution_manager solution_manager

For more information, please refer to the paper. 
