#!/bin/bash

ros2 run experiment_pkg a3_pose_subscriber Sensor upc_laptop 4 3 c 1 60

sleep 10

ros2 run experiment_pkg a3_pose_subscriber Sensor upc_laptop 3 3 c 1 60

sleep 10

ros2 run experiment_pkg a3_pose_subscriber Sensor upc_laptop 2 3 c 1 60

sleep 10

ros2 run experiment_pkg a3_pose_subscriber Sensor upc_laptop 1 3 c 1 60

sleep 10

ros2 run experiment_pkg a3_pose_subscriber Sensor upc_laptop 0 3 c 1 60
