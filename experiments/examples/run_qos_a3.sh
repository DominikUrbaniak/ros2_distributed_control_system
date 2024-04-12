#!/bin/bash
echo "Start a3 pose subscriber"
. install/setup.bash

counter=140

timeout 110s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a3_pose_subscriber R10 100 onboard_PC_cyclone
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a3_pose_subscriber R1 100 onboard_PC_cyclone
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a3_pose_subscriber B10 100 onboard_PC_cyclone
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run push_control_py a3_pose_subscriber B1 100 onboard_PC_cyclone
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a3_pose_subscriber R10 100 onboard_PC_fastrtps
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a3_pose_subscriber R1 100 onboard_PC_fastrtps
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a3_pose_subscriber B10 100 onboard_PC_fastrtps
sleep 30s
timeout 110s RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run push_control_py a3_pose_subscriber B1 100 onboard_PC_fastrtps
