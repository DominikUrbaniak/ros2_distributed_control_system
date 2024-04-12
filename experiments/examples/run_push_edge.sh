#!/bin/bash
echo "Start experiment"
. install/setup.bash
timeout 5s ros2 run experiments push_edge_gazebo

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_edge_gazebo 0.1
    timeout 5s ros2 run experiments push_edge_main 0.1 private5g 0.0001 2 49 #1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_edge_gazebo 0.1
    timeout 5s ros2 run experiments push_edge_main 0.1 private5g_urllc 0.0001 2 49
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_edge_gazebo 0.1
    timeout 5s ros2 run experiments push_edge_main 0.1 private5g 0.0001 2 49
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_edge_gazebo 0.1
    timeout 5s ros2 run experiments push_edge_main 0.1 wifi5_ideal 0.0001 3 170
    # Increment the value of n by 1
    (( counter++ ))
done




echo "Finished experiment"
