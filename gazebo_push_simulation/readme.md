## Gazebo Push Simulation

Use the dockerfile provided here or refer to the source code in this repository: [[https://github.com/DominikUrbaniak/qos_tests.git](https://github.com/DominikUrbaniak/gazebo_push_simulation.git)](https://github.com/DominikUrbaniak/gazebo_push_simulation.git) 

requires Rocker: https://github.com/osrf/rocker

- *docker build -t ros_humble_gazebo_push .*
- *rocker --nvidia --x11 --network=host ros_humble_gazebo_push*
