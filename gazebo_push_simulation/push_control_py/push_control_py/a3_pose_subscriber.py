import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
from rclpy.subscription import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId, PoseCommunication
from geometry_msgs.msg import Pose
from push_control_py import qos_profiles
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'R1':qos_profiles.qos_profile_R1,'R10':qos_profiles.qos_profile_R10,'B1':qos_profiles.qos_profile_B1,'B10':qos_profiles.qos_profile_B10,
'RT':qos_profiles.qos_profile_RT,'RV':qos_profiles.qos_profile_RV,'BT':qos_profiles.qos_profile_BT,'BV':qos_profiles.qos_profile_BV}

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')

        self.communication_duration = 15
        self.qos_profile = "R10" #default profile
        self.file_note = ""
        if len(sys.argv)>2:
            self.qos_profile = sys.argv[1]
            self.communication_duration = int(sys.argv[2])
            if len(sys.argv)>3:
                self.file_note = sys.argv[3]
        self.counter = 0
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile} and measurement duration: {self.communication_duration}')
        self.n_data_points = 30*self.communication_duration
        self.latencies = np.zeros([self.n_data_points,7])
        self.subscription = self.create_subscription(
            PoseCommunication,
            'aruco_detection/tag_poses',
            self.pose_callback,
            qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )

    def pose_callback(self, msg):
        found_tags = msg.tag_ids
        tag_poses = msg.poses
        end_time = self.get_clock().now().nanoseconds
        if self.counter < self.n_data_points:
            self.latencies[self.counter,:] = [msg.id_image_published, msg.id_pose_published, msg.stamp_ns_image_published, msg.stamp_ns_image_received, msg.stamp_ns_pose_published, end_time, len(found_tags)]
        else:
            np.savetxt('docs/data/qos_tests/a3_pose_subscriber_'+self.qos_profile+'_'+str(self.communication_duration)+'_'+self.file_note+'.csv', self.latencies, delimiter=',')
            self.get_logger().info(f'Successful pose measurement!, {found_tags}, {tag_poses}')
            rclpy.shutdown()
        self.counter = self.counter + 1




def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
