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
from experiment_pkg import qos_profiles

#from pympler.asizeof import asizeof
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'R1':qos_profiles.qos_profile_R1,'R10':qos_profiles.qos_profile_R10,'B1':qos_profiles.qos_profile_B1,'B10':qos_profiles.qos_profile_B10,
'RT':qos_profiles.qos_profile_RT,'RV':qos_profiles.qos_profile_RV,'BT':qos_profiles.qos_profile_BT,'BV':qos_profiles.qos_profile_BV}
#from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10


class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')

        self.communication_duration = 15
        self.qos_profile = "R10" #default profile
        self.file_note = ""
        image_widths = [320,640,960,1280,1920]
        if len(sys.argv)>2:
            self.qos_profile = sys.argv[1]
            self.communication_duration = int(sys.argv[2])
            if len(sys.argv)==4:
                self.file_note = sys.argv[3]
            elif len(sys.argv)==6:
                self.file_note = f'{sys.argv[3]}_{sys.argv[4]}_{image_widths[int(sys.argv[5])]}'
        self.counter = 0
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile} and measurement duration: {self.communication_duration}')
        self.n_data_points = 30*self.communication_duration
        self.n_required_tags = 3
        self.measurement_length = self.n_required_tags*3*2 + 1 #position and euler angles for each required tag plus num of detected tags
        self.all_tag_poses = np.zeros([self.n_data_points,self.measurement_length])
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
            append_tag_poses = []
            #if len(found_tags) >= self.n_required_tags:
            # Initialize a dictionary to store poses by their IDs
            pose_dict = {}
            # Populate the dictionary with IDs as keys and poses as values
            for tag_id, pose in zip(found_tags, tag_poses):
                pose_dict[tag_id] = pose
            # Initialize an empty list to store the final results
            n_found_tags = 0
            found_all = True
            # Iterate through IDs 0, 1, and 2 and append poses to the result list
            for tag_id in range(self.n_required_tags):
                if tag_id in pose_dict:
                    n_found_tags += 1
                    pose = pose_dict[tag_id]
                    # Extract position and Euler angles from the pose (assuming appropriate methods)
                    position = [pose.position.x,pose.position.y,pose.position.z]
                    euler_angles = self.euler_from_quaternion(pose.orientation)

                    # Append the position and Euler angles as a tuple to the result list
                    append_tag_poses.append((position, euler_angles))
                else:
                    self.get_logger().warn(f'tag_id {tag_id} not found in {found_tags}')
                    found_all = False
                    pass
            #else:
            #    self.get_logger().warn(f'Not three tags were found: {found_tags}')
            #    pass
            if found_all:
                np_append_tag_poses = np.squeeze(np.reshape(np.array(append_tag_poses),(1,self.measurement_length-1)))

            #if len(np_append_tag_poses) == self.measurement_length:
                #self.get_logger().info(f'poses:, {np_append_tag_poses}')
            #self.all_tag_poses[self.counter,:] = np.array([np_append_tag_poses])
                self.all_tag_poses[self.counter,:] = np.hstack((np_append_tag_poses,n_found_tags))
        else:
            np.savetxt('docs/data/pose_accuracy/a3_'+str(self.communication_duration)+'_'+self.file_note+'.csv', self.all_tag_poses, delimiter=',')
            self.get_logger().info(f'Successful pose measurement!, {found_tags}, {tag_poses}')
            rclpy.shutdown()
        self.counter = self.counter + 1

    #from the construct
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
