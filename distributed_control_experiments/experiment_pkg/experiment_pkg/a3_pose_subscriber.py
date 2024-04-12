import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
#from scipy.spatial.transform import Rotation
from rclpy.subscription import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId, PoseCommunication
from geometry_msgs.msg import Pose
from experiment_pkg import qos_profiles
import logging
from pympler.asizeof import asizeof

#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'R1':qos_profiles.qos_profile_R1,'R10':qos_profiles.qos_profile_R10,'B1':qos_profiles.qos_profile_B1,'B10':qos_profiles.qos_profile_B10,
'RT':qos_profiles.qos_profile_RT,'RV':qos_profiles.qos_profile_RV,'BT':qos_profiles.qos_profile_BT,'BV':qos_profiles.qos_profile_BV}

logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    filename='measurement_log.txt',  # Specify the filename
    filemode='a'  # 'w' to overwrite, 'a' to append
)
delimiter = ';'
output_dir = "docs/data/detection/"

log_messages = []

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')

        self.cv_bridge = CvBridge()

        self.qos_profile = "Sensor"
        self.file_note = "default"
        self.communication_duration = 10
        self.n_intermediate_save = 0
        self.intermediate_duration = 0

        image_widths = [320,640,960,1280,1920]
        image_heights = [240,480,540,720,1080]
        self.save_img = 0

        if len(sys.argv)>7:
            self.qos_profile = sys.argv[1]
            self.file_note = sys.argv[2]
            res_id = int(sys.argv[3])
            view = int(sys.argv[4])
            compressed = sys.argv[5]
            self.save_img = int(sys.argv[6])
            self.communication_duration = int(sys.argv[7])

            if len(sys.argv)>8:
                self.n_intermediate_save = int(sys.argv[8])

        self.image_save_time = int(0.2*self.communication_duration)



        # Specify the directory path you want to create
        self.directory_path = output_dir+self.file_note

        # Check if the directory already exists
        if not os.path.exists(self.directory_path):
            # If it doesn't exist, create the directory
            os.makedirs(self.directory_path)
            os.makedirs(self.directory_path+"/images")
            print(f"Directory '{self.directory_path}' created.")
        else:
            print(f"Directory '{self.directory_path}' already exists.")
        self.start_time = self.get_clock().now()

        if self.n_intermediate_save:
            self.intermediate_duration = self.communication_duration/self.n_intermediate_save
        self.counter = 0
        self.init_time = int(time.time())
        self.intermediate_time = self.init_time
        self.first_log = True

        self.logging_active = 1
        self.timeout_factor = 20
        self.timeout = False

        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')
        self.output_log_filename = self.directory_path + "/" + str(self.init_time) + "_" + self.qos_profile + "_" + str(view) + "_" + compressed + "_" + str(image_widths[res_id]) + "_" + str(image_heights[res_id]) + ".csv"
        self.output_img_filename = self.directory_path + "/images/" + str(self.init_time) + "_" + self.qos_profile + "_" + str(view) + "_" + compressed + "_" + str(image_widths[res_id]) + "_" + str(image_heights[res_id]) + ".png"
        self.subscription = self.create_subscription(
            PoseCommunication,
            'aruco_detection/tag_poses',
            self.sub_callback,
            qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )

        self.subscription_img = self.create_subscription(
            Image,
            'camera/image_arucos',
            self.image_callback,
            10
        )

    def image_callback(self, msg):

        if self.save_img:
            end_time = int(time.time())
            if end_time-self.init_time > self.image_save_time:
                self.save_img = 0
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
                cv2.imwrite(self.output_img_filename, cv_image)
                self.get_logger().info(f'Saved image at {self.output_img_filename}')


    #def pose_callback(self, msg):
#        self.get_logger().info(f'Received a message!')
#        found_tags = msg.tag_ids
#        tag_poses = msg.poses
#        end_time = self.get_clock().now().nanoseconds
#        x = 0
#        y = 0
#        z = 0
#        if(len(found_tags) == 1):
#            x = tag_poses[0].position.x
#            y = tag_poses[0].position.y
#            z = tag_poses[0].position.z
#        if self.counter < self.n_data_points:
#            self.latencies[self.counter,:] = [msg.id_image_published, msg.id_pose_published, msg.stamp_ns_image_published, msg.stamp_ns_image_received, msg.stamp_ns_pose_published, end_time, len(found_tags),x,y,z]
#        else:
#            np.savetxt('docs/data/detection_test/a3_pose_subscriber_'+self.qos_profile+'_'+str(self.communication_duration)+'_'+self.file_note+'.csv', self.latencies, delimiter=',')
#            self.get_logger().info(f'Successful pose measurement!, {found_tags}, {tag_poses}')
#            rclpy.shutdown()
#        self.counter = self.counter + 1


    def sub_callback(self, msg):
        #self.get_logger().info(f'Receiving messages!')
        end_time = int(time.time())
        end_time_ns = self.get_clock().now().nanoseconds
        msg_size = asizeof(msg)
        tag_poses = msg.poses
        found_tags = msg.tag_ids
        x = 0
        y = 0
        z = 0
        if(len(found_tags) == 1):
            x = tag_poses[0].position.x
            y = tag_poses[0].position.y
            z = tag_poses[0].position.z

        if self.n_intermediate_save:
            if end_time > self.intermediate_time + self.intermediate_duration:
                self.intermediate_time = end_time
                with open(f'{self.directory_path}/intermediate_{str(int(self.intermediate_time))}.csv', 'w') as log_file:
                    log_file.write('ID Image pub;Time Stamp Image Published;'
                       'Time Stamp Image Received;Time Stamp Pose Published;Time Stamp Pose Received;E2E latency [ms];Tag found;Cube X;Cube Y;Cube Z;Message Size Image;Message Size Poses\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
                self.get_logger().info(f'Intermediate saving...')

        if end_time-self.init_time < self.communication_duration:
            if self.counter % 100 == 0:
                self.get_logger().info(f'Running communication step {self.counter}, duration: {end_time-self.init_time}')
            if self.logging_active:
                log_message = (
                    f"{msg.id_image_published};{msg.stamp_ns_image_published};{msg.stamp_ns_image_received};{msg.stamp_ns_pose_published};{end_time_ns};{(end_time_ns-msg.stamp_ns_image_published)/1000/1000};{len(found_tags)};{x};{y};{z};{msg.msg_size_image};{msg_size}"
                )
                log_messages.append(log_message)

        else:
            if self.logging_active:
                with open(self.output_log_filename, 'w') as log_file:
                    log_file.write('ID Image pub;Time Stamp Image Published;'
                       'Time Stamp Image Received;Time Stamp Pose Published;Time Stamp Pose Received;E2E latency [ms];Tag found;Cube X;Cube Y;Cube Z;Message Size Image;Message Size Poses\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
            self.get_logger().info(f'Successful measurement!')
            raise SystemExit
            #self.destroy_node()
            #rclpy.shutdown()

        self.counter += 1




def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
