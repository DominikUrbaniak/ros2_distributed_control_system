#!/bin/bash

timeout 38 ros2 run qos_pkg poses_a2_sub_pub

sleep 2

timeout 38 ros2 run qos_pkg poses_a2_sub_pub
