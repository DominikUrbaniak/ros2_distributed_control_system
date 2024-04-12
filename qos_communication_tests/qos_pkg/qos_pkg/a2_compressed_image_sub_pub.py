import sys
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import CompressedImageStampId
from custom_interfaces.srv import QosSettings
from geometry_msgs.msg import Pose
from qos_pkg import qos_profiles
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'RV1':qos_profiles.qos_profile_RV1,'RV10':qos_profiles.qos_profile_RV10,'RV100':qos_profiles.qos_profile_RV100,
'BV1':qos_profiles.qos_profile_BV1,'BV10':qos_profiles.qos_profile_BV10,'BV100':qos_profiles.qos_profile_BV100,
'RT1':qos_profiles.qos_profile_RT1,'RT10':qos_profiles.qos_profile_RT10,'RT100':qos_profiles.qos_profile_RT100,
'BT1':qos_profiles.qos_profile_BT1,'BT10':qos_profiles.qos_profile_BT10,'BT100':qos_profiles.qos_profile_BT100}

class QoSImageSubPub(Node):
    def __init__(self):
        super().__init__('qos_image_sub_pub')

        self.cli_settings = self.create_client(QosSettings, 'get_qos_settings')
        while not self.cli_settings.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = QosSettings.Request()
        self.future = self.cli_settings.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.qos_profile = self.future.result().qos_profile
        #self.file_note = ""

        #if len(sys.argv)>1:
        #    self.qos_profile = sys.argv[1]
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')

        self.subscription = self.create_subscription(
            CompressedImageStampId,
            'qos_compressed_image/pub1',
            self.sub_callback,
            qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )
        self.publisher_2 = self.create_publisher(CompressedImageStampId, 'qos_compressed_image/pub2', qos_profile=qos_profiles_dict[self.qos_profile])


    def sub_callback(self, msg):
        #start_time = time.time()
        #start_time = self.get_clock().now().nanoseconds

        msg2 = CompressedImageStampId()
        #msg2.image = msg.image
        msg2.id = msg.id
        msg2.stamp_ns = msg.stamp_ns
        msg2.stamp_ns2 = self.get_clock().now().nanoseconds

        self.publisher_2.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = QoSImageSubPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
