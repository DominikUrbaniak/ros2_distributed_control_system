#!/bin/bash
echo "Start a3 pose subscriber"
. install/setup.bash

echo "Timout after 1s"
#ros2 run push_control_py a1_image_publisher R10 0
timeout 10s ros2 run push_control_py a1_image_publisher R10 0

#echo "Timout after 1"
#timeout 1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a1_image_publisher R10 0

counter=140
