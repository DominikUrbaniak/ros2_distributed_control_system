
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import ImageStampIdSrv
from push_control_py.qos_profiles import qos_profile_R1

current_profile = qos_profile_R1

class ImageService(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_service')
    image_width = 640
    image_height = 480
    fps = 30
    self.counter = 0
    self.stamp_ns = 0
    self.image = Image()
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    #self.publisher_ = self.create_publisher(ImageStampId, 'camera/image_raw', current_profile)
    self.service = self.create_service(ImageStampIdSrv, 'aruco_image_service', self.service_callback, qos_profile=current_profile)
    # We will publish a message every 0.1 seconds
    timer_period = 1.0/fps  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
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
      self.image = self.cv_bridge.cv2_to_imgmsg(frame)
      self.id = self.counter
      self.stamp_ns = self.get_clock().now().nanoseconds#self.get_clock().now()#time.time()
      #self.publisher_.publish(msg)
      self.counter = self.counter + 1
      #end_time = time.time()
      #computation_time = end_time - start_time
      #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))

  def service_callback(self, request, response):
    #self.get_logger().info(f'time after service call start: {time.time()-request.timestamp_in}')
    response.image = self.image
    response.id =  self.counter
    response.stamp_ns = self.stamp_ns
    return response

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_service = ImageService()

  # Spin the node so the callback function is called.
  rclpy.spin(image_service)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_service.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
