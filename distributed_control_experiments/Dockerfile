FROM ros:humble-ros-base
RUN apt-get update 
RUN apt-get install -y python3-pip ros-humble-cv-bridge 
RUN pip install opencv-contrib-python==4.5.4.60 scipy pympler 
RUN pip install "numpy<2" mediapipe
WORKDIR /workspace/src 
RUN git clone https://github.com/DominikUrbaniak/ur_experiments.git
WORKDIR /workspace/docs/data 
WORKDIR /workspace 
RUN . /opt/ros/humble/setup.sh && \ 
  colcon build
