FROM ros:humble 
RUN apt-get update 
RUN curl -sSL http://get.gazebosim.org | sh
RUN apt-get install -y python3-pip ros-humble-cv-bridge 
RUN pip install opencv-contrib-python==4.5.4.60 scipy pympler 
RUN apt-get install -y ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-joint-state-broadcaster ros-humble-velocity-controllers
WORKDIR /workspace/src 
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b humble
RUN git clone https://gitioc.upc.edu/robots/kinenik.git
RUN mkdir tes02
RUN git clone https://github.com/DominikUrbaniak/gazebo_push_simulation.git
WORKDIR /workspace 
RUN . /opt/ros/humble/setup.sh && \ 
  colcon build
