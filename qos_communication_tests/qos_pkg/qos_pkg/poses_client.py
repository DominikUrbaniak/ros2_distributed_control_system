import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import PosesStampIdSrv
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from pympler.asizeof import asizeof
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    filename='measurement_log.txt',  # Specify the filename
    filemode='a'  # 'w' to overwrite, 'a' to append
)
delimiter = ';'
output_dir = "docs/data/poses_srv/"

log_messages = []

class PosesClientNode(Node):
    def __init__(self):
        super().__init__('poses_client_node')
        self.counter = 0

        self.qos_profile = "RV10" #default profile
        self.file_note = "default"
        self.fps = 100
        self.packet_size = 1000
        unit_size = 4.3
        min_size = 260
        self.communication_duration = 10
        self.timeout_factor = 20
        self.timeout = False
        self.init_time = int(time.time())
        self.stamp_server = 0
        self.id = 0
        self.start_time = self.get_clock().now().nanoseconds

        if len(sys.argv)>1:
            self.communication_duration = int(sys.argv[1])
            if len(sys.argv)>2:
                self.file_note = sys.argv[2]
                if len(sys.argv)>3:
                    self.fps = int(sys.argv[3])
                    if len(sys.argv)>4:
                        self.packet_size = int(sys.argv[4])
        if self.packet_size<min_size:
            self.get_logger().warn(f'Min packet size is ~260 Bytes')
            self.num_poses = 1
        else:
            self.num_poses = int(np.round((self.packet_size-min_size)/unit_size))

        self.get_logger().info(f'Starting measurement @{self.fps} Hz, file note: {self.file_note}, num poses: {self.num_poses}')
        directory_path = output_dir+self.file_note

        # Check if the directory already exists
        if not os.path.exists(directory_path):
            # If it doesn't exist, create the directory
            os.makedirs(directory_path)
            print(f"Directory '{directory_path}' created.")
        else:
            print(f"Directory '{directory_path}' already exists.")
        self.output_log_filename = directory_path + "/" + str(self.init_time) + "_" +str(self.fps)+"_" +str(self.packet_size)+"_"+ self.qos_profile + ".csv"

        # We will publish a message every 0.1 seconds
        timer_period = 1.0/self.fps  # seconds

        self.n_data_points = self.fps*self.communication_duration

        self.counter = 0


        # Create separate callback groups
        self.timer1_callback_group = MutuallyExclusiveCallbackGroup()
        self.service1_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the timer in the timer callback group
        self.timer1 = self.create_timer(1.0/self.fps, self.timer_callback1, callback_group=self.timer1_callback_group)

        # Create the client in the service callback group
        self.client1 = self.create_client(PosesStampIdSrv, '/poses_srv', callback_group=self.service1_callback_group)

        #found_aruco = False

    def timer_callback1(self):

        start_time_ns = self.get_clock().now().nanoseconds

        #self.get_logger().info(f'python time: {start_time} vs. ROS2 time: {start_time_ros2} vs. time stamp: {msg.stamp_ns}')
        #msg = ImageStampId()
        # Convert ROS Image message to OpenCV format
        if not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service /poses_srv not available. Trying again...')
            return

        request = PosesStampIdSrv.Request()

        request.poses = np.zeros(self.num_poses, dtype=np.int32).tolist()
        request.stamp_ns = start_time_ns
        request.id = self.counter
        request.qos_profile = self.qos_profile
        msg_size_req = asizeof(request)
        response = self.client1.call(request)
        msg_size_res = asizeof(response)
        self.poses = response.poses
        self.id = response.id
        self.qos_profile = response.qos_profile
        self.stamp_server = response.stamp_ns
        end_time_ns = self.get_clock().now().nanoseconds
        #pub_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        if (end_time_ns - self.start_time) > self.communication_duration * self.timeout_factor * 1000*1000*1000:
            with open(self.output_log_filename, 'w') as log_file:
                log_file.write('ID; Stamp published; Stamp server; Stamp received; E2E latency [ms]; Message size req]; Message size res\n')
                for message in log_messages:
                    log_file.write(message + '\n')
            self.get_logger().info(f'Interrupted by timeout!')
            raise SystemExit

        if self.counter < self.n_data_points:
            if self.counter % 100 == 0:
                self.get_logger().info(f'Running communication step {self.counter}/{self.n_data_points}')

            log_message = (
                    f"{self.id};{start_time_ns};{self.stamp_server};{end_time_ns};{(end_time_ns-start_time_ns)/1000/1000};{msg_size_req};{msg_size_res}"
            )
            log_messages.append(log_message)
        else:
            with open(self.output_log_filename, 'w') as log_file:
                log_file.write('ID; Stamp published; Stamp server; Stamp received; E2E latency [ms]; Message size\n')
                for message in log_messages:
                    log_file.write(message + '\n')
            self.get_logger().info(f'Successful measurement!')
            #raise SystemExit
            rclpy.shutdown()
        #self.get_logger().info(f'Time: {end_time - start_time}')
        #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))
        #self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        self.counter = self.counter + 1


def main(args=None):
    rclpy.init(args=args)
    node = PosesClientNode()
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
