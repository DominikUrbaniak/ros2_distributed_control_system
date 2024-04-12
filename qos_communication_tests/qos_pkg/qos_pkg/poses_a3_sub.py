import sys
import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
import logging
#import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import PosesStampId
from custom_interfaces.srv import QosSettings
from geometry_msgs.msg import Pose
from qos_pkg import qos_profiles

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
output_dir = "docs/data/poses/"

log_messages = []

class QoSPosesSub(Node):
    def __init__(self):
        super().__init__('qos_poses_sub')

        self.communication_duration = 10
        self.n_intermediate_save = 0
        if len(sys.argv)>1:
            self.communication_duration = int(sys.argv[1])
            if len(sys.argv)>2:
                self.n_intermediate_save = int(sys.argv[2])
        self.cli_settings = self.create_client(QosSettings, 'get_qos_settings')
        cc = 0
        cli_call = 1
        while not self.cli_settings.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            cc += 1
            if cc > 10:
                cli_call = 0
                self.get_logger().info('service was not available')
                break

        if cli_call:

            self.req = QosSettings.Request()
            self.future = self.cli_settings.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            self.qos_profile = self.future.result().qos_profile
            self.sensing_rate = self.future.result().sensing_rate
            self.packet_size = self.future.result().packet_size
            self.file_note = self.future.result().file_note
            self.msg_size_ul = self.future.result().msg_size
        else:
            if len(sys.argv)>7:
                self.qos_profile = sys.argv[3]
                self.file_note = sys.argv[4]
                self.sensing_rate = int(sys.argv[5])
                self.packet_size = int(sys.argv[6])
                self.msg_size_ul = int(sys.argv[7])
            else:
                self.get_logger().info('service not reachable, set the params in the command line')
                return

        # Specify the directory path you want to create
        self.directory_path = output_dir+self.file_note

        # Check if the directory already exists
        if not os.path.exists(self.directory_path):
            # If it doesn't exist, create the directory
            os.makedirs(self.directory_path)
            print(f"Directory '{self.directory_path}' created.")
        else:
            print(f"Directory '{self.directory_path}' already exists.")
        self.start_time = self.get_clock().now()
        self.counter = 0
        self.init_time = int(time.time())
        self.first_log = True

        self.logging_active = 1
        self.timeout_factor = 20
        self.timeout = False




        self.n_data_points = self.sensing_rate*self.communication_duration
        if self.n_intermediate_save:
            self.save_ids = int(self.n_data_points/self.n_intermediate_save)
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')
        self.output_log_filename = self.directory_path + "/" + str(self.init_time) + "_" +str(self.sensing_rate)+"_" +str(self.packet_size)+"_"+ self.qos_profile + ".csv"
        self.subscription = self.create_subscription(
            PosesStampId,
            'qos_poses/pub2',
            self.sub_callback,
            qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )

    def sub_callback(self, msg):
        end_time = int(time.time())
        end_time_ns = self.get_clock().now().nanoseconds
        msg_size = asizeof(msg)

        if self.n_intermediate_save:
            if self.counter > 0 and self.counter%self.save_ids == 0:
                with open(f'{self.directory_path}/intermediate_{str(int(time.time()))}.csv', 'w') as log_file:
                    log_file.write('ID;Stamp published;Stamp server;Stamp received;E2E latency [ms];Message size UL;Message size DL\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
                self.get_logger().info(f'Intermediate saving...')

        if end_time-self.init_time < self.communication_duration:
            if self.counter % 100 == 0:
                self.get_logger().info(f'Running communication step {self.counter}, duration: {end_time-self.init_time}/{self.communication_duration}')
            if self.logging_active:
                log_message = (
                    f"{msg.id};{msg.stamp_ns};{msg.stamp_ns2};{end_time_ns};{(end_time_ns-msg.stamp_ns)/1000/1000};{self.msg_size_ul};{msg_size}"
                )
                log_messages.append(log_message)

        else:
            if self.logging_active:
                with open(self.output_log_filename, 'w') as log_file:
                    log_file.write('ID;Stamp published;Stamp server;Stamp received;E2E latency [ms];Message size UL;Message size DL\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
            self.get_logger().info(f'Successful measurement!')
            raise SystemExit
            #self.destroy_node()
            #rclpy.shutdown()

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = QoSPosesSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
