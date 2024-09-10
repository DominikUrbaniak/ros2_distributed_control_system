# Distributed Control System using ROS 2 

This repository contains three experiments which have been developed and performed at IOC-UPC, Roboception and the 5G Smart Production Lab at Aalborg University:

1) a robotic push experiment simulated with Gazebo11 that is delayed by a sample latency from measurements at Aalborg University (can be run in docker/rocker containers) (https://github.com/DominikUrbaniak/gazebo_push_simulation.git).
   
2) ROS 2 QoS communication tests where the best-effort and reliable QoS profiles can be compared for customized different package sizes uplink (can be run in docker containers) (https://github.com/DominikUrbaniak/qos_tests.git).

3) the real distrbuted control experiments using a UR5e, offloading image processing of arUco detection and hand landmark detection to the edge server, receiving the object poses to control the robot using the ROS 2 JointGroupVelocityController (https://github.com/DominikUrbaniak/ur_experiments.git).

[![Mira el video](https://github.com/DominikUrbaniak/ros2_distributed_control_system/blob/main/teleoperation.png)](https://github.com/DominikUrbaniak/ros2_distributed_control_system/blob/main/teleoperation_5G_comp_comp.mp4)
