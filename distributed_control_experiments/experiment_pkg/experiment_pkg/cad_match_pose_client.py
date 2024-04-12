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
from rc_reason_msgs.srv import CadMatchDetectObject
from rc_reason_msgs.msg import Tag
from geometry_msgs.msg import Pose
from experiment_pkg import qos_profiles
import logging
from pympler.asizeof import asizeof
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

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
output_dir = "docs/data/rc_tag_detect/"

log_messages = []

class RcPoseClient(Node):
    def __init__(self):
        super().__init__('rc_pose_client')

        self.file_note = "default"
        self.communication_duration = 10
        self.n_intermediate_save = 0
        self.intermediate_duration = 0
        self.fps = 25

        if len(sys.argv)>2:
            self.file_note = sys.argv[1]
            self.communication_duration = int(sys.argv[2])
            if len(sys.argv)>3:
                self.n_intermediate_save = int(sys.argv[3])



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

        if self.n_intermediate_save:
            self.intermediate_duration = self.communication_duration/self.n_intermediate_save
        self.counter = 0
        self.init_time = int(time.time())
        self.intermediate_time = self.init_time
        self.first_log = True

        self.logging_active = 1
        self.timeout_factor = 20
        self.timeout = False

        self.get_logger().info(f'Starting measurement...')
        self.output_log_filename = self.directory_path + "/" + str(self.init_time) + ".csv"

        # Create separate callback groups
        self.timer1_callback_group = MutuallyExclusiveCallbackGroup()
        self.service1_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the timer in the timer callback group
        self.timer1 = self.create_timer(1.0/self.fps, self.timer_callback, callback_group=self.timer1_callback_group)

        # Create the client in the service callback group
        self.client1 = self.create_client(CadMatchDetectObject, '/rc_cadmatch_client/detect_object', callback_group=self.service1_callback_group)

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
#            self.latencies[self.counter,:] = [msg.id_image_published, msg.id_pose_published, msg.stamp_ns_image_published, msg.stamp_ns_image_received, msg.stamp_ns_pose_published, res_time, len(found_tags),x,y,z]
#        else:
#            np.savetxt('docs/data/detection_test/a3_pose_subscriber_'+self.qos_profile+'_'+str(self.communication_duration)+'_'+self.file_note+'.csv', self.latencies, delimiter=',')
#            self.get_logger().info(f'Successful pose measurement!, {found_tags}, {tag_poses}')
#            rclpy.shutdown()
#        self.counter = self.counter + 1


    def timer_callback(self):
        #self.get_logger().info(f'Receiving messages!')
        req_time = int(time.time())
        req_time_ns = self.get_clock().now().nanoseconds

        if not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service /rc_cadmatch_client/detect_object not available. Trying again...')
            return
        #tag = Tag()
        request = CadMatchDetectObject.Request()

        request.template_id = "test_template_SCS10UU_ICP"
        request.pose_frame = "camera"

        msg_size_req = asizeof(request)
        response = self.client1.call(request)
        msg_size_res = asizeof(response)
        res_time_ns = self.get_clock().now().nanoseconds


        grasps = response.grasps
        #found_tags = msg.tag_ids
        x = 0
        y = 0
        z = 0
        if(len(response.grasps) == 1):
            x = response.grasps[0].pose.position.x
            y = response.grasps[0].pose.position.y
            z = response.grasps[0].pose.position.z

        if self.n_intermediate_save:
            if req_time > self.intermediate_time + self.intermediate_duration:
                self.intermediate_time = req_time
                with open(f'{self.directory_path}/intermediate_{str(int(self.intermediate_time))}.csv', 'w') as log_file:
                    log_file.write('Time Stamp Pose Requested;Time Stamp Pose Received;E2E latency [ms];Tag found;Cube X;Cube Y;Cube Z;Message Size Image;Message Size Poses\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
                self.get_logger().info(f'Intermediate saving...')

        if req_time-self.init_time < self.communication_duration:
            if self.counter % 100 == 0:
                self.get_logger().info(f'Running communication step {self.counter}, duration: {req_time-self.init_time}')
            if self.logging_active:
                log_message = (
                    f"{req_time_ns};{res_time_ns};{(res_time_ns-req_time_ns)/1000/1000};{len(response.grasps)};{x};{y};{z};{msg_size_req};{msg_size_res}"
                )
                log_messages.append(log_message)

        else:
            if self.logging_active:
                with open(self.output_log_filename, 'w') as log_file:
                    log_file.write('Time Stamp Pose Requested;Time Stamp Pose Received;E2E latency [ms];Tag found;Cube X;Cube Y;Cube Z;Message Size Image;Message Size Poses\n')
                    for message in log_messages:
                        log_file.write(message + '\n')
            self.get_logger().info(f'Successful measurement!')
            #raise SystemExit
            self.destroy_node()
            rclpy.shutdown()

        self.counter += 1




def main(args=None):
    rclpy.init(args=args)
    node = RcPoseClient()
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
