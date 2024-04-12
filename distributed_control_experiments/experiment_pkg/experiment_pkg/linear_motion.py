import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose#, TransformStamped

from custom_interfaces.srv import RobPose


import time
import math
import random
import numpy as np
import configparser
import logging


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')


        self.cli_control = self.create_client(RobPose, '/real_ur3e_velocity_controller/set_cart_dir')

        while not self.cli_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = RobPose.Request()

        request.cart_pose = 0
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

        request.timeout_ms = survival_time_ms

        request.goal_dir = [vx,vy,vz,0.0,0.0,0.0]

        future = self.cli_control.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('request successful!')
            global time_stamp_eef, manipulability_index, eef_x, eef_y, eef_yaw, timeout_triggered, time_since_last_req_ms
            time_stamp_eef = future.result().time_stamp
            manipulability_index = future.result().manipulability_index
            eef_x = future.result().eef_x
            eef_y = future.result().eef_y
            eef_yaw = future.result().eef_yaw
            timeout_triggered = future.result().timeout_triggered
            time_since_last_req_ms = future.result().time_since_last_req_ms
        else:
            self.get_logger().warning('Failed to control')



def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)

    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
