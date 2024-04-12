import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import ImageStampIdSrv
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rc_reason_msgs.srv import ComputeGrasps
from rc_reason_msgs.msg import Item
from geometry_msgs.msg import Pose

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

current_profile = qos_profile_B10
counter1 = 0
class RcItemPickNode(Node):
    def __init__(self):
        super().__init__('rc_itempick_detection_node')
        self.get_logger().info(f'Starting camera...')

        #self.publisher_ = self.create_publisher(Image, 'camera/image_arucos', 10)
        self.communication_rate = 5
        self.communication_duration = 30
        self.n_data_points = self.communication_rate*self.communication_duration
        self.latencies = np.zeros([self.n_data_points,4])
        self.counter = 0
        self.item_found = 0


        # Create separate callback groups
        self.timer1_callback_group = MutuallyExclusiveCallbackGroup()
        self.service1_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the timer in the timer callback group
        self.timer1 = self.create_timer(1.0/self.communication_rate, self.timer_callback1, callback_group=self.timer1_callback_group)

        # Create the client in the service callback group
        self.client1 = self.create_client(ComputeGrasps, '/rc_itempick_client/compute_grasps', callback_group=self.service1_callback_group)

        #found_aruco = False

    def timer_callback1(self):
        start_time = time.time()
        #start_time_ros2 = self.get_clock().now()
        #self.get_logger().info(f'python time: {start_time} vs. ROS2 time: {start_time_ros2} vs. time stamp: {msg.stamp_ns}')
        #msg = ImageStampId()
        # Convert ROS Image message to OpenCV format
        if not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service1 not available.')
            return
        global counter1
        counter1 = counter1 + 1
        request = ComputeGrasps.Request()
        request.suction_surface_width = 0.02
        request.suction_surface_length = 0.02
        request.pose_frame = "camera"
        #request.region_of_interest_id = "cube_roi_01"
        response = self.client1.call(request)
        #detected_tag = DetectedTag()
        detected_items = response.items
        cube_pose = Pose()
        if detected_items:
            self.item_found = 1
            cube_pose = detected_items[0].pose
        else:
            self.item_found = 0
        stamp = response.timestamp.sec*1e9 + response.timestamp.nanosec
        #pub_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        latency = self.get_clock().now().nanoseconds - stamp
        #latency = latency_stamp.sec * 1e9 + latency_stamp.nanosec
        #counter_id = response.id



        #else:
            #found_aruco = False

        end_time = time.time()
        computation_time = int((end_time - start_time)*1e9) #nanosecs
        if self.counter < self.n_data_points:
            self.latencies[self.counter,:] = [self.counter, latency, self.item_found, computation_time]
        else:
            np.savetxt('docs/data/rc_itempick_latency_measurement_srv.csv', self.latencies, delimiter=',')
            self.get_logger().info(f'Successful measurement! Pose: {cube_pose}')
            rclpy.shutdown()
        #self.get_logger().info(f'Time: {end_time - start_time}')
        #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))
        #self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        self.counter = self.counter + 1


def main(args=None):
    rclpy.init(args=args)
    node = RcItemPickNode()
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
