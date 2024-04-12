#!/bin/bash

timeout 405900 ros2 run qos_pkg poses_a2_sub_pub

sleep 100

timeout 405900 ros2 run qos_pkg poses_a2_sub_pub
