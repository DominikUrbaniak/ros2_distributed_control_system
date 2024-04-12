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

class ArUcoTagDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        #self.subscription_callbacks = SubscriptionEventCallbacks(
    #        deadline=self.sub_deadline_event
#        )


        self.publisher_ = self.create_publisher(Image, 'camera/image_arucos', 10)

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
        self.cube_poses = []
        self.tag_ids = []

        self.counter = 0
        self.tag_found = 0
        self.qos_profile = "R10" #default profile
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
                ImageStampId,
                'camera/image_raw',
                self.image_callback,
                qos_profiles_dict[self.qos_profile]#,event_callbacks=self.subscription_callbacks
            )
        self.pose_publisher_ = self.create_publisher(PoseCommunication, 'aruco_detection/tag_poses', qos_profile=qos_profiles_dict[self.qos_profile])



        #found_aruco = False

    #def sub_deadline_event(self, event):
    #    count = event.total_count
    #    delta = event.total_count_change
    #    self.get_logger().info(f'Requested deadline missed - total {count} delta {delta}')

    def image_callback(self, msg):
        #start_time = time.time()
        start_time = self.get_clock().now().nanoseconds
        #self.get_logger().info(f'python time: {start_time} vs. ROS2 time: {start_time_ros2} vs. time stamp: {msg.stamp_ns}')
        #msg = ImageStampId()
        # Convert ROS Image message to OpenCV format
        if self.rc:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        else:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg.image)

        #stamp_ns_image_received = self.get_clock().now().nanoseconds
        #pub_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        #(msg.header.stamp.seconds * 1e9 + msg.header.stamp.nanoseconds)
        #latency = latency_stamp.sec * 1e9 + latency_stamp.nanosec


        # Convert the image to grayscale for ArUco detection
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the dictionary of ArUco tags
        aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Create the ArUco detector
        aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Detect ArUco tags
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, aruco_dictionary, parameters=aruco_parameters)
        #self.get_logger().info(f'corners: {corners}, ids: {ids}, K: {self.cameraMatrix.shape}')
        n_detected_tags = 0
        self.cube_poses = []
        self.tag_ids = []
        #self.tag_found = 0
        if ids is not None:
            ids_list = np.squeeze(ids,axis=1).tolist()
            #self.get_logger().info(f'ids_list: {ids_list}')
            n_detected_tags = len(ids)
            # Find the index of the ArUco tag with ID 0
            #tag_index = np.where(ids == 0)[0]

            # Calculate the pose of the detected tag
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.034, self.cameraMatrix, self.distortionCoeffs)

            #if len(tag_index) > 0:
            for i in range(n_detected_tags):
                #if 1:

                    # Publish the pose of the tag
                    # Get the rotation vector and translation vector of the tag
                #rvec = rvecs[tag_index]
                #tvec = tvecs[tag_index]
                rvec = rvecs[i]
                tvec = tvecs[i]
                if rvec is not None and tvec is not None:

                    #self.tag_found = 1
                    self.tag_ids.append(ids)
                    #self.get_logger().info(f'ArUco tag found, ids: {ids}')
                    #for i in enumerate(ids):
                        #cv2.aruco.drawAxis(cv_image,self.cameraMatrix,self.distortionCoeffs,rvecs[i],tvecs[i],0.1)
                    cv2.aruco.drawAxis(cv_image,self.cameraMatrix,self.distortionCoeffs,rvec,tvec,0.1)
                    rmat, _ = cv2.Rodrigues(rvec)
                    # Transform the tag's position to the camera frame
                    #self.tag_position_camera = -np.dot(rmat, np.transpose(tvec[0]))
                    # Convert the rotation matrix to Euler angles
                    #self.euler_angles = Rotation.from_matrix(rmat).as_euler('xyz', degrees=True)
                    #tag_quat = Rotation.from_matrix(rmat).as_quat()
                    # Transform the tag's position to the world frame
                    #tag_position_world = np.dot(rmat_world_camera, tag_position_camera)
                    pose_camera_matrix = self.create_pose_matrix(rmat, tvec)
                    pose_world_matrix = np.dot(self.transformation_matrix, pose_camera_matrix)
                    pose_world = self.matrix_to_pose(pose_world_matrix)
                    #self.get_logger().info(f'Position: {self.tag_position_camera} vs Pose position: {pose_world.position}, Orientation: {tag_quat} vs Pose.orientation: {pose_world.orientation}')
                    self.cube_poses.append(pose_world)
                    #cube_pose.position = Pose.position(self.tag_position_camera)
                    # Publish the pose information using ROS
                    # Replace 'tag_pose_topic' with the actual topic name for publishing pose data
                    # Replace 'camera_frame' and 'tag_frame' with the actual frame names
                    # Publish rvecs and tvecs
        else:
            ids_list = []

        poses_msg = PoseCommunication()
        poses_msg.poses = self.cube_poses
        poses_msg.id_pose_published = self.counter
        poses_msg.tag_ids = ids_list
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

        if self.logging and not self.rc:
            if self.counter < self.n_data_points:
                self.latencies[self.counter,:] = [msg.id]
            else:
                np.savetxt('docs/data/qos_tests/a2_id_'+self.qos_profile+'_'+str(self.communication_duration)+'_'+self.file_note+'.csv', self.latencies, delimiter=',')
                self.get_logger().info(f'Successful image measurement!')
                rclpy.shutdown()
        #self.get_logger().info(f'Tag position: {self.tag_position_camera}, tag orientation: {self.euler_angles}')
        #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))
        #self.get_logger().info(f'Counter id: {counter_id}, latency: {latency}')
        self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        self.counter = self.counter + 1



        '''def pose_to_matrix(self, pose):
            return np.array([
                [pose.orientation.w, -pose.orientation.z, pose.orientation.y, pose.position.x],
                [pose.orientation.z, pose.orientation.w, -pose.orientation.x, pose.position.y],
                [-pose.orientation.y, pose.orientation.x, pose.orientation.w, pose.position.z],
                [0, 0, 0, 1]
            ])'''

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
    node = ArUcoTagDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
