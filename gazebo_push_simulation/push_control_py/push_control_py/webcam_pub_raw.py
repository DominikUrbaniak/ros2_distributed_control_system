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
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId
# Define QoS profiles
qos_profile_R10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_ALL,         # Keep all messages KEEP_ALL
    depth=10                                      # Keep 10 messages in history 10
)
qos_profile_R1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1                                      # Keep one message in history 1
)
qos_profile_B10 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_ALL,         # Keep all messages KEEP_ALL
    depth=10                                      # Keep 10 messages in history 10
)
qos_profile_B1 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1                                      # Keep one message in history 1
)
current_profile = qos_profile_R1

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
    image_width = 640
    image_height = 480
    fps = 30
    self.counter = 0
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'camera/image_raw2', current_profile)

    # We will publish a message every 0.1 seconds
    timer_period = 1.0/fps  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    camera_id = 0
    if len(sys.argv) > 1:
        camera_id = int(sys.argv[1])
    self.get_logger().info(f'Selected camera id: {camera_id}')
    self.cap = cv2.VideoCapture(camera_id)
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
    self.cap.set(cv2.CAP_PROP_FPS, fps)

    # Used to convert between ROS and OpenCV images
    self.cv_bridge = CvBridge()
    #self.cameraMatrix = 1000*np.array([[1.6695,0.0,0.9207],[0.0,1.6718,0.5518],[0,0,0.0010]]) #Logitech Desktop webcam
    #self.distortionCoeffs = np.array([0.0772,-0.2883,0.0,0.0]) #k1,k2,p1,p2

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    start_time = time.time()
    ret, frame = self.cap.read()

    if ret == True:
      #msg = ImageStampId()
      msg = self.cv_bridge.cv2_to_imgmsg(frame)
      #msg.id = self.counter
      #msg.stamp_ns = self.get_clock().now().nanoseconds#self.get_clock().now()#time.time()
      self.publisher_.publish(msg)
      self.counter = self.counter + 1
      #end_time = time.time()
      #computation_time = end_time - start_time
      self.get_logger().info(f'Sending image #{self.counter}')



def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_publisher = ImagePublisher()

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
