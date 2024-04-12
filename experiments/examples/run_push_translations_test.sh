#!/bin/bash
echo "Start experiment"
. install/setup.bash

ros2 run push_control_py sim_translation ethernet 120 1
echo "Finished ethernet 1"
sleep 1
ros2 run push_control_py sim_translation wifi5_loaded 120 1
echo "Finished wifi5_loaded 1"
sleep 1
ros2 run push_control_py sim_translation ethernet 120 1
echo "Finished ethernet 2"




echo "Finished experiment"
