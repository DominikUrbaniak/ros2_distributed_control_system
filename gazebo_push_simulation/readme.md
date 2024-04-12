## Gazebo Push Simulation

This package includes a simulated push experiment with a UR5e and a gazebo service that receives the pose of the object which is delayed by a latency sample from some communication technology, measured at Aalborg University.

I requries running a launch file,
ros2 launch main_pkg main.launch.py

and a node that takes the name of the communication technology, the sensing rate and the number of episodes as inputs:
ros2 run push_control_py sim_translation private5g 120 200

Further parameters can be set in main_pkg/config/sim_translation.ini, such as the low-level control timeout and pushing speed.
