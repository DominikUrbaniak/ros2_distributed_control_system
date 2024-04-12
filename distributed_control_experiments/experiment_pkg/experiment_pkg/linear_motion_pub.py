import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose#, TransformStamped

from custom_interfaces.msg import VelDir


import time
import math
import random
import numpy as np
import configparser
import logging


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.publisher_ = self.create_publisher(VelDir, '/vel_dirs', rclpy.qos.qos_profile_sensor_data)
        #self.timer_ = self.create_timer(1.0 / 30, self.publish_vel_dir)
        self.publisher_active_ = False
        msg = VelDir()


        vx = 0.0
        vy = 0.1
        vz = 0.0
        survival_time_ms = 100
        if len(sys.argv)>3:
            vx = float(sys.argv[1])
            vy = float(sys.argv[2])
            vz = float(sys.argv[3])
            if len(sys.argv)>4:
                survival_time_ms = int(sys.argv[4])

        msg.timeout_ms = survival_time_ms
        msg.goal_dir = [vx,vy,vz,0.0,0.0,0.0]

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)

    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
