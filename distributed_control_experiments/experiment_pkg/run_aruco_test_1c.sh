#!/bin/bash

timeout 70 ros2 run experiment_pkg a1_compressed_image_publisher Sensor 9 30 4

timeout 70 ros2 run experiment_pkg a1_compressed_image_publisher Sensor 9 30 3

timeout 70 ros2 run experiment_pkg a1_compressed_image_publisher Sensor 9 30 2

timeout 70 ros2 run experiment_pkg a1_compressed_image_publisher Sensor 9 30 1

timeout 70 ros2 run experiment_pkg a1_compressed_image_publisher Sensor 9 30 0
