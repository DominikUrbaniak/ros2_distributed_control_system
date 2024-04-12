#!/bin/bash
echo "Start experiment"
. install/setup.bash

ros2 run push_control_py sim_translation wifi5_loaded 120 200
echo "Finished wifi5_loaded 120 50"
ros2 run push_control_py sim_translation wifi5_ideal 120 200
echo "Finished wifi5_ideal 120 50"
ros2 run push_control_py sim_translation private5g 120 200
echo "Finished private5g 120 50"
ros2 run push_control_py sim_translation private5g_urllc 120 200
echo "Finished private5g_urllc 120 50"
ros2 run push_control_py sim_translation private4g 120 200
echo "Finished private4g 120 50"
ros2 run push_control_py sim_translation ethernet 120 200
echo "Finished ethernet 120 50"

ros2 run push_control_py sim_translation wifi5_loaded 240 200
echo "Finished wifi5_loaded 240 50"
ros2 run push_control_py sim_translation wifi5_ideal 240 200
echo "Finished wifi5_ideal 240 50"
ros2 run push_control_py sim_translation private5g 240 200
echo "Finished private5g 240 50"
ros2 run push_control_py sim_translation private5g_urllc 240 200
echo "Finished private5g_urllc 240 50"
ros2 run push_control_py sim_translation private4g 240 200
echo "Finished private4g 240 50"
ros2 run push_control_py sim_translation ethernet 240 200
echo "Finished ethernet 240 50"

ros2 run push_control_py sim_translation wifi5_loaded 960 200
echo "Finished wifi5_loaded 960 50"
ros2 run push_control_py sim_translation wifi5_ideal 960 200
echo "Finished wifi5_ideal 960 50"
ros2 run push_control_py sim_translation private5g 960 200
echo "Finished private5g 960 50"
ros2 run push_control_py sim_translation private5g_urllc 960 200
echo "Finished private5g_urllc 960 50"
ros2 run push_control_py sim_translation private4g 960 200
echo "Finished private4g 960 50"
ros2 run push_control_py sim_translation ethernet 960 200
echo "Finished ethernet 960 50"

ros2 run push_control_py sim_translation wifi5_loaded 60 200
echo "Finished wifi5_loaded 60 50"
ros2 run push_control_py sim_translation wifi5_ideal 60 200
echo "Finished wifi5_ideal 60 50"
ros2 run push_control_py sim_translation private5g 60 200
echo "Finished private5g 60 50"
ros2 run push_control_py sim_translation private5g_urllc 60 200
echo "Finished private5g_urllc 60 50"
ros2 run push_control_py sim_translation private4g 60 200
echo "Finished private4g 60 50"
ros2 run push_control_py sim_translation ethernet 60 200
echo "Finished ethernet 60 50"

ros2 run push_control_py sim_translation wifi5_loaded 30 200
echo "Finished wifi5_loaded 30 50"
ros2 run push_control_py sim_translation wifi5_ideal 30 200
echo "Finished wifi5_ideal 30 50"
ros2 run push_control_py sim_translation private5g 30 200
echo "Finished private5g 30 50"
ros2 run push_control_py sim_translation private5g_urllc 30 200
echo "Finished private5g_urllc 30 50"
ros2 run push_control_py sim_translation private4g 30 200
echo "Finished private4g 30 50"
ros2 run push_control_py sim_translation ethernet 30 200
echo "Finished ethernet 30 50"

ros2 run push_control_py sim_translation wifi5_loaded 480 200
echo "Finished wifi5_loaded 480 50"
ros2 run push_control_py sim_translation wifi5_ideal 480 200
echo "Finished wifi5_ideal 480 50"
ros2 run push_control_py sim_translation private5g 480 200
echo "Finished private5g 480 50"
ros2 run push_control_py sim_translation private5g_urllc 480 200
echo "Finished private5g_urllc 480 50"
ros2 run push_control_py sim_translation private4g 480 200
echo "Finished private4g 480 50"
ros2 run push_control_py sim_translation ethernet 480 200
echo "Finished ethernet 480 50"


echo "Finished experiment"
