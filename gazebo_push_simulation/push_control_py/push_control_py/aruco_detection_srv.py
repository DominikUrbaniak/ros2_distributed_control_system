import sys
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.srv import PoseDetection
from geometry_msgs.msg import Pose
from push_control_py import qos_profiles

qos_profiles_dict = {'R1':qos_profiles.qos_profile_R1, 'R5':qos_profiles.qos_profile_R5, 'R10':qos_profiles.qos_profile_R10, 'R20':qos_profiles.qos_profile_R20,
    'RT1':qos_profiles.qos_profile_RT1, 'RT10':qos_profiles.qos_profile_RT10, 'RV10':qos_profiles.qos_profile_RV10, 'RV1':qos_profiles.qos_profile_RV1}

current_profile = qos_profiles.qos_profile_R1

class ImageService(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_service')
        image_width = 640
        image_height = 480
        fps = 30
        self.counter = 0
        self.stamp_ns = 0
        self.qos_profile = "R10" #default profile
        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]
        self.cv_bridge = CvBridge()
        self.cameraMatrix = 1000*np.array([[1.6695,0.0,0.9207],[0.0,1.6718,0.5518],[0,0,0.0010]]) #Logitech Desktop webcam
        self.distortionCoeffs = np.array([0.0772,-0.2883,0.0,0.0]) #k1,k2,p1,p2
        # Define the transformation matrix from camera frame to world frame
        self.transformation_matrix = np.eye(4)  # Update with your actual transformation matrix
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        #self.publisher_ = self.create_publisher(ImageStampId, 'camera/image_raw', current_profile)
        self.service = self.create_service(PoseDetection, 'aruco_image_service', self.service_callback, qos_profile=qos_profiles_dict[self.qos_profile])
        # We will publish a message every 0.1 seconds
        self.get_logger().info(f'Starting service with qos_profile: {self.qos_profile}')

    def service_callback(self, request, response):
        #self.get_logger().info(f'time after service call start: {time.time()-request.timestamp_in}')
        start_time = self.get_clock().now().nanoseconds
        #self.get_logger().info(f'start time srv: {start_time}')
        cv_image = self.cv_bridge.imgmsg_to_cv2(request.image)
        counter_id = request.id
        stamp_image_requested = request.stamp_ns
        stamp_diff = start_time - stamp_image_requested
        #self.get_logger().info(f'start time srv {counter_id}: {start_time}, diff: {stamp_diff}')
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
        end_time = self.get_clock().now().nanoseconds
        computation_time = end_time - start_time
        response.poses = self.cube_poses
        response.id_response = counter_id
        response.tag_ids = ids_list
        response.stamp_ns_pose_response = end_time
        response.stamp_ns_image_request = stamp_image_requested
        response.stamp_ns_image_received = start_time
        response.qos_profile = self.qos_profile
        return response

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

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_service = ImageService()

  # Spin the node so the callback function is called.
  rclpy.spin(image_service)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_service.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
