import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from rclpy.subscription import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import CompressedImageStampId, PoseCommunication
from geometry_msgs.msg import Pose
from experiment_pkg import qos_profiles

from pympler.asizeof import asizeof

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'R1':qos_profiles.qos_profile_R1,'R10':qos_profiles.qos_profile_R10,'B1':qos_profiles.qos_profile_B1,'B10':qos_profiles.qos_profile_B10,
'RT':qos_profiles.qos_profile_RT,'RV':qos_profiles.qos_profile_RV,'BT':qos_profiles.qos_profile_BT,'BV':qos_profiles.qos_profile_BV}

class HandLandmarkDetectionNode(Node):
    def __init__(self):
        super().__init__('hand_detection_node')


        self.cv_bridge = CvBridge()
        self.cameraMatrix = 1000*np.array([[1.6695,0.0,0.9207],[0.0,1.6718,0.5518],[0,0,0.0010]]) #Logitech Desktop webcam
        self.distortionCoeffs = np.array([0.0772,-0.2883,0.0,0.0]) #k1,k2,p1,p2
        # Define the transformation matrix from camera frame to world frame
        self.transformation_matrix = np.eye(4)  # Update with your actual transformation matrix
        self.tag_position_camera = np.zeros(3)
        self.euler_angles = np.zeros(3)
        self.communication_duration = 0
        self.n_data_points = 0
        self.latencies = np.zeros([self.n_data_points,1])
        self.logging = False
        self.landmark_poses = []
        self.tag_ids = []

        self.counter = 0
        self.tag_found = 0
        self.qos_profile = "Sensor" #default profile
        self.file_note = ""
        self.rc = 0
        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]
            if len(sys.argv)>2:
                self.rc = int(sys.argv[2])
                if len(sys.argv)>3:
                    self.communication_duration = int(sys.argv[3])
                    if len(sys.argv)>4:
                        self.file_note = sys.argv[4]
        self.get_logger().info(f'Starting measurement with rc?->{self.rc}, with qos_profile: {self.qos_profile}, logging: {self.logging}, measurement duration: {self.communication_duration}')
        if self.communication_duration != 0:
            self.logging = True
            self.n_data_points = 30*self.communication_duration
            self.latencies = np.zeros([self.n_data_points,1])
        topic_name = 'camera/image_raw'
        if self.rc:
            topic_name = '/stereo/left/image_rect'
            self.subscription = self.create_subscription(
                Image,
                '/stereo/left/image_rect',
                self.image_callback,
                qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
            )
        else:
            self.subscription = self.create_subscription(
                CompressedImageStampId,
                'camera/image_raw',
                self.image_callback,
                qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
            )
        self.pose_publisher_ = self.create_publisher(PoseCommunication, 'hand_detection/landmark_poses', qos_profile=qos_profiles_dict[self.qos_profile])
        self.publisher_ = self.create_publisher(Image, 'camera/image_hand_landmarks', 10)

    def image_callback(self, msg):
        #start_time = time.time()
        start_time = self.get_clock().now().nanoseconds
        msg_size_image = asizeof(msg)
        #self.get_logger().info(f'python time: {start_time} vs. ROS2 time: {start_time_ros2} vs. time stamp: {msg.stamp_ns}')
        #msg = ImageStampId()
        # Convert ROS Image message to OpenCV format
        if self.rc:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        else:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg.image)

        self.landmark_poses = []
        with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
                max_num_hands=1) as hands:

            cv_image = cv2.flip(cv_image, 1)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            cv_image.flags.writeable = False
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = hands.process(cv_image)

            # Draw the hand annotations on the image.
            cv_image.flags.writeable = True
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            c1 = 0
            c2 = 0
            #if results.multi_hand_landmarks:
            if results.multi_hand_landmarks:
                #for hand_landmarks in results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    c1 += 1
                    for landmark in hand_landmarks.landmark:
                        c2 +=1
                        #self.get_logger().info(f'hand_landmarks: {landmark}, {c1}, {c2}')
                        landmark_pose = Pose()
                        landmark_pose.position.x = landmark.x
                        landmark_pose.position.y = landmark.y
                        landmark_pose.position.z = landmark.z
                        self.landmark_poses.append(landmark_pose)
                    #landmark_pose.position.x =
                    mp_drawing.draw_landmarks(
                        cv_image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())


            else:
                landmark_pose = Pose()
                self.landmark_poses.append(landmark_pose)

        poses_msg = PoseCommunication()
        poses_msg.poses = self.landmark_poses
        poses_msg.id_pose_published = self.counter
        poses_msg.msg_size_image = msg_size_image

        if self.rc:
            poses_msg.id_image_published = 0
            stamp_ns = int(msg.header.stamp.sec*10e8) + msg.header.stamp.nanosec
            poses_msg.stamp_ns_image_published = stamp_ns
        else:
            poses_msg.id_image_published = msg.id
            poses_msg.stamp_ns_image_published = msg.stamp_ns
        poses_msg.stamp_ns_image_received = start_time

        end_time = self.get_clock().now().nanoseconds
        poses_msg.stamp_ns_pose_published = end_time
        self.pose_publisher_.publish(poses_msg)

        #self.get_logger().info(f'Tag position: {self.tag_position_camera}, tag orientation: {self.euler_angles}')
        #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))
        #self.get_logger().info(f'Counter id: {counter_id}, latency: {latency}')
        #self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        self.counter = self.counter + 1


    def create_pose_matrix(self, rmat, tvec):
        # Create a 4x4 transformation matrix
        pose_matrix = np.eye(4)
        # Fill the top-left 3x3 submatrix with the rotation matrix
        pose_matrix[:3, :3] = rmat
        # Fill the rightmost column with the translation vector
        pose_matrix[:3, 3] = tvec
        return pose_matrix

    def matrix_to_pose(self, matrix):
        pose = Pose()
        pose.orientation.w = matrix[0, 0]
        pose.orientation.x = matrix[1, 1]
        pose.orientation.y = matrix[2, 2]
        pose.orientation.z = matrix[3, 3]
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = HandLandmarkDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
