#!/bin/bash
echo "Start experiment"
. install/setup.bash

ros2 run push_control_py sim_translation wifi5_loaded 60 200
echo "Finished wifi5_loaded 60 50"




echo "Finished experiment"
