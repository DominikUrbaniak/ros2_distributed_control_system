import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import PoseDetection
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class ArUcoTagDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.get_logger().info(f'Starting camera...')

        #self.publisher_ = self.create_publisher(Image, 'camera/image_arucos', 10)
        image_width = 640
        image_height = 480
        fps = 30
        self.counter = 0
        camera_id = 0
        self.communication_duration = 15
        self.file_note = ""
        if len(sys.argv)>1:
            camera_id = int(sys.argv[1])
            if len(sys.argv)>2:
                self.communication_duration = int(sys.argv[2])
                if len(sys.argv)>3:
                    self.file_note = sys.argv[3]
        self.get_logger().info(f'Starting measurement with camera id: {camera_id}, duration: {self.communication_duration}, note: {self.file_note}')
        # We will publish a message every 0.1 seconds
        timer_period = 1.0/fps  # seconds

        self.n_data_points = fps*self.communication_duration
        self.latencies = np.zeros([self.n_data_points,8])
        self.counter = 0
        self.tag_found = 0
        self.tag_poses = []
        self.tag_ids = []
        self.stamp_srv_request = 0
        self.stamp_srv_response = 0
        self.stamp_srv_received = 0
        self.id_srv = 0
        self.qos_profile = "R10"

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Used to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()

        # Create separate callback groups
        self.timer1_callback_group = MutuallyExclusiveCallbackGroup()
        self.service1_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the timer in the timer callback group
        self.timer1 = self.create_timer(1.0/fps, self.timer_callback1, callback_group=self.timer1_callback_group)

        # Create the client in the service callback group
        self.client1 = self.create_client(PoseDetection, 'aruco_image_service', callback_group=self.service1_callback_group)

        #found_aruco = False

    def timer_callback1(self):

        #start_time_ros2 = self.get_clock().now()
        #self.get_logger().info(f'python time: {start_time} vs. ROS2 time: {start_time_ros2} vs. time stamp: {msg.stamp_ns}')
        #msg = ImageStampId()
        # Convert ROS Image message to OpenCV format
        if not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('aruco_image_service not available. Trying again...')
            return

        ret, frame = self.cap.read()
        start_time_client = self.get_clock().now().nanoseconds
        #self.get_logger().info(f'start time client: {start_time_client}')

        if ret == True:
            request = PoseDetection.Request()
            request.image = self.cv_bridge.cv2_to_imgmsg(frame)
            request.stamp_ns = start_time_client
            request.id = self.counter

            response = self.client1.call(request)

            self.tag_poses = response.poses
            self.tag_ids = response.tag_ids
            self.stamp_srv_response = response.stamp_ns_pose_response
            self.id_srv = response.id_response
            self.stamp_srv_request = response.stamp_ns_image_request
            self.stamp_srv_received = response.stamp_ns_image_received
            self.qos_profile = response.qos_profile


        #pub_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        end_time_client = self.get_clock().now().nanoseconds


        if self.counter < self.n_data_points:
            self.latencies[self.counter,:] = [self.counter, self.id_srv, start_time_client, self.stamp_srv_received, self.stamp_srv_response, end_time_client, len(self.tag_ids)]
        else:
            np.savetxt('docs/data/qos_tests/a01_latency_measurement_srv_'+self.qos_profile+'_'+str(self.communication_duration)+'_'+self.file_note+'.csv', self.latencies, delimiter=',')
            self.get_logger().info(f'Successful measurement!')
            rclpy.shutdown()
        #self.get_logger().info(f'Time: {end_time - start_time}')
        #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))
        #self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        self.counter = self.counter + 1


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoTagDetectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    #executor.add_node(control_node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    executor.shutdown()


if __name__ == '__main__':
    main()
