# ROS 2 Distributed Control System

This repository links three experiments:

1) a robotic push experiment simulated with Gazebo11 that is delayed by a sample latency from measurements at Aalborg University.
   
2) ROS 2 QoS communication tests where the best-effort and reliable QoS profiles can be compared for customized different package sizes uplink (can be run in docker containers) (https://github.com/DominikUrbaniak/qos_tests.git)

3) the real distrbuted control experiments using a UR5e, offloading image processing of arUco detection and hand landmark detection to the edge server, receiving the object poses to control the robot using the ROS 2 jointgroupvelocity controller (https://github.com/DominikUrbaniak/ur_experiments.git)
