## QoS Communication Tests

Communication via ROS2 using publisher/subscriber mechanism and reliable or best-effort reliability QoS profiles. Variable size of packets can be set, as well as the communication rate. A docker file is provided for easier usage.

## External Installations
- OpenCV version 4.5.4.60

## Packages in this repository
- qos_pkg: contains the publisher and subscriber nodes that can be placed in the distributed computer network.
- custom_interfaces: Includes the custom messages and services

## Docker
The nodes can be run using Docker containers. 
## Run experiment

- *ros2 run qos_pkg poses_a1_pub Sensor rc_ethernet 60 20000 1*

QoS profile: best effort ( *Sensor* ) or reliable ( *RV10* ), folder name to save measurements ( *rc_ethernet* ), approximate packet size in bytes ( *20000* ), mode: ( *0*: same packet size uplink and downlink, *1*: specified packet size only uplink, *2*: specified packet size only downlink
- *ros2 run qos_pkg poses_a2_sub_pub*
- *ros2 run qos_pkg poses_a3_sub 60* Duration of the measurement ( *60* )



