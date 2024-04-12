import sys
import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
import logging
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId
from custom_interfaces.srv import QosSettings
from geometry_msgs.msg import Pose
from qos_pkg import qos_profiles
from sensor_msgs.msg import Image
from pympler.asizeof import asizeof
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'RV1':qos_profiles.qos_profile_RV1,'RV10':qos_profiles.qos_profile_RV10,'RV100':qos_profiles.qos_profile_RV100,
'BV1':qos_profiles.qos_profile_BV1,'BV10':qos_profiles.qos_profile_BV10,'BV100':qos_profiles.qos_profile_BV100,
'RT1':qos_profiles.qos_profile_RT1,'RT10':qos_profiles.qos_profile_RT10,'RT100':qos_profiles.qos_profile_RT100,
'BT1':qos_profiles.qos_profile_BT1,'BT10':qos_profiles.qos_profile_BT10,'BT100':qos_profiles.qos_profile_BT100}

logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    filename='measurement_log.txt',  # Specify the filename
    filemode='a'  # 'w' to overwrite, 'a' to append
)
delimiter = ';'
output_dir = "docs/data/image_one_way/"

log_messages = []

class QoSImageSub(Node):
    def __init__(self):
        super().__init__('qos_image_sub')

        self.cli_settings = self.create_client(QosSettings, 'get_qos_settings')
        while not self.cli_settings.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = QosSettings.Request()
        self.future = self.cli_settings.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        self.qos_profile = self.future.result().qos_profile
        self.sensing_rate = self.future.result().sensing_rate
        self.image_width = self.future.result().image_width
        self.image_height = self.future.result().image_height
        self.file_note = self.future.result().file_note
        # Specify the directory path you want to create
        #self.qos_profile = "RV10"
        #self.file_note = "default"
        #self.resolution = "axb"
        #self.sensing_rate = 0
        # Check if the directory already exists

        self.counter = 0
        self.init_time = int(time.time())
        self.first_log = True
        self.communication_duration = 10
        self.start_time = self.get_clock().now()
        self.logging_active = 1
        self.timeout_factor = 20
        self.timeout = False
        if len(sys.argv)>1:
            self.communication_duration = int(sys.argv[1])

        directory_path = output_dir+self.file_note
        if not os.path.exists(directory_path):
            # If it doesn't exist, create the directory
            os.makedirs(directory_path)
            print(f"Directory '{directory_path}' created.")
        else:
            print(f"Directory '{directory_path}' already exists.")
        self.n_data_points = self.sensing_rate*self.communication_duration
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')
        self.output_log_filename = directory_path + "/" + str(self.init_time) + "_" +str(self.sensing_rate)+"_" +str(self.image_width)+"_"+str(self.image_height)+"_"+ self.qos_profile + ".csv"
        self.subscription = self.create_subscription(
            ImageStampId,
            'qos_image/pub2',
            self.sub_callback,
            qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )
        self.publisher_ = self.create_publisher(Image, 'rs_camera/comrpessed_image', 10)

    def sub_callback(self, msg):
        end_time = self.get_clock().now().nanoseconds
        msg_size = asizeof(msg)

        if (end_time - self.start_time.nanoseconds) > self.communication_duration * self.timeout_factor * 1000*1000*1000:
            if self.logging_active:
                with open(self.output_log_filename, 'w') as log_file:
                    log_file.write('ID;Stamp published;Stamp server;Stamp received;E2E latency [ms];Message size\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
            self.get_logger().info(f'Interrupted by timeout!')
            raise SystemExit

        if self.counter < self.n_data_points:
            if self.counter % 100 == 0:
                self.get_logger().info(f'Running communication step {self.counter}/{self.n_data_points}')
            if self.logging_active:
                log_message = (
                    f"{msg.id};{msg.stamp_ns};{msg.stamp_ns2};{end_time};{(end_time-msg.stamp_ns)/1000/1000};{msg_size}"
                )
                log_messages.append(log_message)
            self.publisher_.publish(msg.image)

        else:
            if self.logging_active:
                with open(self.output_log_filename, 'w') as log_file:
                    log_file.write('ID; Stamp published; Stamp server;Stamp received; E2E latency [ms]; Message size\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
            self.get_logger().info(f'Successful measurement!')
            raise SystemExit
            #self.destroy_node()
            #rclpy.shutdown()

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = QoSImageSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
