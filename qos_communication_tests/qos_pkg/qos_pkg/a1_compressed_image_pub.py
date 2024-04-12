# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
import sys
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#from std_msgs.msg import Empty # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
#import cv2 # OpenCV library
import numpy as np
import time
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import CompressedImageStampId
from custom_interfaces.srv import QosSettings
from qos_pkg import qos_profiles
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'RV1':qos_profiles.qos_profile_RV1,'RV10':qos_profiles.qos_profile_RV10,'RV100':qos_profiles.qos_profile_RV100,
'BV1':qos_profiles.qos_profile_BV1,'BV10':qos_profiles.qos_profile_BV10,'BV100':qos_profiles.qos_profile_BV100,
'RT1':qos_profiles.qos_profile_RT1,'RT10':qos_profiles.qos_profile_RT10,'RT100':qos_profiles.qos_profile_RT100,
'BT1':qos_profiles.qos_profile_BT1,'BT10':qos_profiles.qos_profile_BT10,'BT100':qos_profiles.qos_profile_BT100}

class QoSPublisher1(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('qos_publisher_1')

    self.counter = 0
    self.qos_profile = "RV10" #default profile
    self.file_note = "default"
    self.fps = 100
    self.image_width = 1280#1920
    self.image_height = 720#1080
    self.camera_id = 0
    if len(sys.argv)>1:
        self.qos_profile = sys.argv[1]
        if len(sys.argv)>2:
            camera_id = int(sys.argv[2])
            if len(sys.argv)>3:
                self.file_note = sys.argv[3]
                if len(sys.argv)>4:
                    self.fps = int(sys.argv[4])
                    if len(sys.argv)>6:
                        self.image_width = int(sys.argv[5])
                        self.image_height = int(sys.argv[6])
    self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile} @{self.fps} Hz, camera id: {self.camera_id}, file note: {self.file_note}, image size: {self.image_width} x {self.image_height}')
    self.publisher_ = self.create_publisher(CompressedImageStampId, 'qos_compressed_image/pub1', qos_profiles_dict[self.qos_profile])#,event_callbacks=self.publisher_callbacks

    # We will publish a message every 0.1 seconds
    timer_period = 1.0/self.fps  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(self.camera_id)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    # Used to convert between ROS and OpenCV images
    self.cv_bridge = CvBridge()
    self.service = self.create_service(QosSettings, 'get_qos_settings', self.service_callback)

  def service_callback(self, request, response):
      response.qos_profile = self.qos_profile
      response.sensing_rate = self.fps
      response.image_height = self.image_height
      response.image_width = self.image_width
      response.file_note = self.file_note
      return response

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.


    ret, frame = self.cap.read()
    start_time = self.get_clock().now().nanoseconds

    if ret == True:
      msg = CompressedImageStampId()
      # Convert the ROS2 image message to a compressed image message
      #msg.image = self.cv_bridge.cv2_to_imgmsg(frame)
      msg.image = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
      msg.id = self.counter
      msg.stamp_ns = start_time#self.get_clock().now()#time.time()
      self.publisher_.publish(msg)
      self.counter = self.counter + 1
      #end_time = time.time()
      #computation_time = end_time - start_time
      #self.get_logger().info(f'Sending image #{self.counter}')



def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_publisher = QoSPublisher1()

  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
