#!/bin/bash
echo "Start a2 aruco detection"
. install/setup.bash

timeout 120s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a2_aruco_detection R10 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a2_aruco_detection R1 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a2_aruco_detection B10 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a2_aruco_detection B1 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a2_aruco_detection R10 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a2_aruco_detection R1 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a2_aruco_detection B10 0
sleep 20s
timeout 120s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a2_aruco_detection B1 0
