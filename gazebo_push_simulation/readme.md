## Gazebo Push Simulation

Use the Dockerfile provided here or refer to the source code in this repository: https://github.com/DominikUrbaniak/gazebo_push_simulation.git

requires Rocker for rendering the Gazebo simulation: https://github.com/osrf/rocker

In Dockerfile repository run:
- *docker build -t ros_humble_gazebo_push .*

Run in two terminals:
- *rocker --nvidia --x11 --network=host ros_humble_gazebo_push*
- *. install/setup.bash*

Terminal 1:
- *ros2 launch main_pkg main.launch.py*

Terminal 2:
- *ros2 run push_control_py sim_translation private5g 60 10*
