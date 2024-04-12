#!/bin/bash
echo "Start a1 image publisher"
. install/setup.bash

timeout 130s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a1_image_publisher R10 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a1_image_publisher R1 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a1_image_publisher B10 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a1_image_publisher B1 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a1_image_publisher R10 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a1_image_publisher R1 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a1_image_publisher B10 0
sleep 10s
timeout 130s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a1_image_publisher B1 0
