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
from push_control_py import qos_profiles
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'R1':qos_profiles.qos_profile_R1,'R10':qos_profiles.qos_profile_R10,'B1':qos_profiles.qos_profile_B1,'B10':qos_profiles.qos_profile_B10,
'RT':qos_profiles.qos_profile_RT,'RV':qos_profiles.qos_profile_RV,'BT':qos_profiles.qos_profile_BT,'BV':qos_profiles.qos_profile_BV}

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
    #self.publisher_callbacks = PublisherEventCallbacks(
#        deadline=self.pub_deadline_event
#    )
    camera_id = 0
    self.qos_profile = "R10" #default profile
    if len(sys.argv)>1:
        self.qos_profile = sys.argv[1]
        if len(sys.argv) > 2:
            camera_id = int(sys.argv[2])
            if len(sys.argv) > 3:
                fps = int(sys.argv[3])
                if len(sys.argv) > 5:
                    image_width = int(sys.argv[4])
                    image_height = int(sys.argv[5])
    self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}, camera id: {camera_id}, fps: {fps}, image resolution {image_width}x{image_height}')
    self.publisher_ = self.create_publisher(ImageStampId, 'camera/image_raw', qos_profiles_dict[self.qos_profile])#,event_callbacks=self.publisher_callbacks

    # We will publish a message every 0.1 seconds
    timer_period = 1.0/fps  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(camera_id)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
    self.cap.set(cv2.CAP_PROP_FPS, fps)



    # Used to convert between ROS and OpenCV images
    self.cv_bridge = CvBridge()
    #self.cameraMatrix = 1000*np.array([[1.6695,0.0,0.9207],[0.0,1.6718,0.5518],[0,0,0.0010]]) #Logitech Desktop webcam
    #self.distortionCoeffs = np.array([0.0772,-0.2883,0.0,0.0]) #k1,k2,p1,p2

  #def pub_deadline_event(self, event):
    #count = event.total_count
    #delta = event.total_count_change
    #self.get_logger().info(f'Requested deadline missed - total {count} delta {delta}')

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
      msg = ImageStampId()
      msg.image = self.cv_bridge.cv2_to_imgmsg(frame)
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
