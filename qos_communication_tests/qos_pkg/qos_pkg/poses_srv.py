import sys
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
#import cv2 # OpenCV library
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import PosesStampIdSrv
from geometry_msgs.msg import Pose
from qos_pkg import qos_profiles
from std_srvs.srv import Empty

from pympler.asizeof import asizeof
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'R10':qos_profiles.qos_profile_R10,'RV1':qos_profiles.qos_profile_RV1,'RV10':qos_profiles.qos_profile_RV10,'RV100':qos_profiles.qos_profile_RV100,
'RT1':qos_profiles.qos_profile_RT1,'RT10':qos_profiles.qos_profile_RT10,'RT100':qos_profiles.qos_profile_RT100}

class PosesService(Node):
    def __init__(self):
        super().__init__('poses_service')

        self.qos_profile = "RV10"
        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]

        #self.publisher_ = self.create_publisher(ImageStampId, 'camera/image_raw', current_profile)
        self.service = self.create_service(PosesStampIdSrv, '/poses_srv', self.service_callback, qos_profile=qos_profiles_dict[self.qos_profile])
        self.service2 = self.create_service(Empty, '/poses_test_srv', self.service_callback2, qos_profile=qos_profiles_dict[self.qos_profile])
        # We will publish a message every 0.1 seconds
        self.get_logger().info(f'Starting service /poses_srv with qos_profile: {self.qos_profile}')

    def service_callback(self, request, response):
        #self.get_logger().info(f'time after service call start: {time.time()-request.timestamp_in}')
        start_time = self.get_clock().now().nanoseconds
        #self.get_logger().info(f'start time srv: {start_time}')
        response.poses = request.poses
        response.id = request.id
        response.stamp_ns = start_time
        response.qos_profile = self.qos_profile
        return response

    def service_callback2(self, request, response):
        self.get_logger().info(f'Service called successfully')
        return response

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  node = PosesService()

  # Spin the node so the callback function is called.
  rclpy.spin(node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  node.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
