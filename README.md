# Robotic Arm
This repository contains the code of the robotic arm handler node, and the world file for the Gazebo simulation.

This project has been implemented with ROS1 Noetic on Ubuntu 20.04. It requires MoveIt, [ros_kortex](https://github.com/NorbertPap/my_gen3_lite) (forked from [the original ros_kortex library from Kinova](https://github.com/Kinovarobotics/ros_kortex)), and [Gazebo-Gripper-Action-Controller](https://github.com/ian-chuang/Gazebo-Gripper-Action-Controller) installed in the same catkin workspace as this package.

## ArmHandlingNode
The source file for the only node in this package, called ArmHandlingNode is located in [/src/arm_handling_node.py](/src/arm_handling_node.py).

## Action interfaces
The ArmHandlingNode provides two actions with interfaces under [/action](/action).

## Launch files
There are two launch files in this package under [/launch](/launch). The file [demo.launch](/launch/demo.launch) is used to launch the node itself, whereas [my_world.launch](/launch/my_world.launch) is used to change the default empty world launch file included in the ros_kortex library's spawn_kortex_robot.launch file to the custom world of this package.

## World file
The world file used for the simulation of this application can be found under [/worlds](/worlds). It contains two custom boxes, and 3 tennis balls that have been changed from spheres to now be boxes instead.

### Note - Under construction
Since this code hasn't been integrated with the other packages of the project, the actions haven't been tested by actually sending action requests to this node, but only by calling the callbacks in the main function in [/src/arm_handling_node.py](/src/arm_handling_node.py)