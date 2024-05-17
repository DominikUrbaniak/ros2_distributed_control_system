## Gazebo Push Simulation

Use the dockerfile provided here or refer to the source code in this repository: https://github.com/DominikUrbaniak/gazebo_push_simulation.git

requires Rocker for rendering the Gazebo simulation: https://github.com/osrf/rocker

- *docker build -t ros_humble_gazebo_push .*
- *rocker --nvidia --x11 --network=host ros_humble_gazebo_push*

Terminal 1:
- *. install/setup.bash*
- *ros2 launch main_pkg main.launch.py*

Terminal 2:
- *. install/setup.bash*
- *ros2 run push_control_py sim_translation private5g 60 10*
