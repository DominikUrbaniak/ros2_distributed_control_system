import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import Pose#, TransformStamped
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from custom_interfaces.srv import RobPose
from std_msgs.msg import Float64MultiArray
#from custom_interfaces.srv import SetRandomization
from custom_interfaces.srv import ResetPoses, String
from custom_interfaces.msg import PoseSensing, VelDir
from custom_interfaces.srv import PoseSensingSettings
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId, PoseCommunication
from experiment_pkg import qos_profiles
from rc_reason_msgs.srv import DetectTags
from rc_reason_msgs.msg import Tag

qos_profiles_dict = {'Sensor':rclpy.qos.qos_profile_sensor_data,'R1':qos_profiles.qos_profile_R1,'R10':qos_profiles.qos_profile_R10,'B1':qos_profiles.qos_profile_B1,'B10':qos_profiles.qos_profile_B10,
'RT':qos_profiles.qos_profile_RT,'RV':qos_profiles.qos_profile_RV,'BT':qos_profiles.qos_profile_BT,'BV':qos_profiles.qos_profile_BV}

import time
import math
import random
import numpy as np
import configparser
import logging
from pympler.asizeof import asizeof

def set_config_params(filename):
    global const_vel_x,const_vel_y,goal_x,goal_y,goal_yaw,flip_goal_dir_sign,sensing_rate,qos_profile,push_distance,survival_time_ms,experiment_timeout,perform_logging,push_direction,tag_id
    config = configparser.ConfigParser()
    config.read(filename)
    section = 'General'
    config_sec = config[section]
    #go_const_vel = config_sec.getboolean('go_const_vel', True)
    const_vel_x = config_sec.getfloat('const_vel_x', 0.05)
    const_vel_y = config_sec.getfloat('const_vel_y', 0.00)
    goal_x = config_sec.getfloat('goal_x', 0.00)
    goal_y = config_sec.getfloat('goal_y', 0.00)
    goal_yaw = config_sec.getfloat('goal_yaw', 0.00)
    flip_goal_dir_sign = config_sec.getboolean('flip_goal_dir_sign', True)
    #backup_ms = config_sec.getint('backup_ms', 200) #for backing up when cube target reached
    #network = config_sec.get('network', 'private5g')
    #computation_ms = config_sec.getint('computation_ms', 0)
    #n_episode = config_sec.getint('n_episode', 5)

    sensing_rate = config_sec.getint('sensing_rate', 25)
    qos_profile = config_sec.get('quality_of_service', 'R10')
    push_distance = config_sec.getfloat('push_distance', 0.04)
    #n_freshness_samples = config_sec.getint('n_freshness_samples', 10)
    survival_time_ms = config_sec.getint('survival_time_ms', 200) #for low-level control
    experiment_timeout = config_sec.getint('experiment_timeout', 10)
    perform_logging = config_sec.getboolean('perform_logging', False)
    push_direction = config_sec.get('push_direction', 'y') #'x', 'y', 'yaw'
    tag_id = config_sec.getint('tag_id', 0)

def set_goal_position():
    global goal_position
    if push_direction == 'x':
        goal_position = goal_x
    elif push_direction == 'yaw':
        goal_position = goal_yaw
    elif push_direction == 'y':
        goal_position = goal_y
    else:
        self.get_logger().warn(f'Specified push direction: {push_direction} is unknown, please select x, y or yaw ')
    return goal_position

keys_control = []
values_control = []

cube_pose = Pose()
cube_y_init = 0.58

v_init = 0.01
v_z_offset = 0.0#0.009#0.011
v_x_offset = 0.0#-0.001
manipulability_index = 1
time_stamp_eef = 0
eef_x = 0.0
eef_y = 0.0
eef_yaw = 0.0
timeout_triggered = 0
time_since_last_req_ms = 0
timeout_triggered
accuracy = 0.0
execution_time = 0.0
#timeout_ms = 200

n_speed_values = 1000
config_filename = 'src/control_pkg/config/rc_real_pose_real_push.ini'
#config = read_config(config_filename)
#config = configparser.ConfigParser()
#config.read(config_filename)
#section = 'General'
#config_sec = config[section]
#go_const_vel = config_sec.getboolean('go_const_vel', True)
const_vel_x = 0.05 #config_sec.getfloat('const_vel_x', 0.05)
const_vel_y = 0.0 #config_sec.getfloat('const_vel_y', 0.00)
goal_position = 0.0 #config_sec.getfloat('goal_position', 0.00)
goal_x = 0.0 #config_sec.getfloat('goal_position', 0.00)
goal_y = 0.0 #config_sec.getfloat('goal_position', 0.00)
goal_yaw = 0.0 #config_sec.getfloat('goal_position', 0.00)
flip_goal_dir_sign = False #config_sec.getboolean('flip_goal_dir_sign', True)
#backup_ms = 200 #config_sec.getint('backup_ms', 200) #for backing up when cube target reached
#network = config_sec.get('network', 'private5g')
#computation_ms = config_sec.getint('computation_ms', 0)
#n_episode = config_sec.getint('n_episode', 5)
sensing_rate = 25 #config_sec.getint('sensing_rate', 30)
qos_profile = 'Sensor' #config_sec.get('quality_of_service', 'Sensor')
#push_distance = config_sec.getfloat('push_distance', 0.04)
#n_freshness_samples = config_sec.getint('n_freshness_samples', 10)
survival_time_ms = 200 #config_sec.getint('survival_time_ms', 200) #for low-level control
experiment_timeout = 10 #config_sec.getint('experiment_timeout', 10)
perform_logging = False #config_sec.getboolean('perform_logging', False)
push_direction = 'y' #config_sec.get('push_direction', 'y') #'x', 'y', 'yaw'
tag_id = 0 #config_sec.getint('tag_id', 0)

set_config_params(config_filename)
start_time = int(time.time())

general_keys = ["const_vel_x","const_vel_y","flip_goal_dir_sign","push_direction","quality_of_service","sensing_rate","survival_time_ms","experiment_timeout","tag_id","goal_x","goal_y","goal_yaw","perform_logging"]


time_since_last_req_ms = 0.0
eef_stamp = 0.0
eef_x = 0.0
eef_y = 0.0
eef_z = 0.0
eef_roll = 0.0
eef_pitch = 0.0
eef_yaw = 0.0
folder = "default"

if len(sys.argv)>1:
    folder = sys.argv[1]
    if len(sys.argv)>2:
        qos_profile = sys.argv[2]
        if len(sys.argv)>3:
            sensing_rate = int(sys.argv[3])
            if len(sys.argv)>4:
                survival_time_ms = int(sys.argv[4])
                if len(sys.argv)>4:
                    experiment_timeout = int(sys.argv[4])

output_dir = "docs/data/logging_rc/"+folder+"/"
if not os.path.exists(output_dir):
    # If it doesn't exist, create the directory
    os.makedirs(output_dir)
    print(f"Directory '{output_dir}' created.")
else:
    print(f"Directory '{output_dir}' already exists.")

vx = const_vel_x
vy = const_vel_y
vz = 0.0
vqr = 0.0
vqp = 0.0
vqy = 0.0

history = np.array([])
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    filename='measurement_log.txt',  # Specify the filename
    filemode='w'  # 'w' to overwrite, 'a' to append
)
delimiter = ';'

log_messages = []


#from the construct
def euler_from_quaternion(quaternion):
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

    return roll, pitch, yaw

import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return Quaternion(w=w, x=x, y=y, z=z)

class RealPoseRealPush(Node):

    def __init__(self):
        super().__init__('real_pose_real_push')
        self.get_logger().info(f'Starting push experiment...')
        #random.seed()
        #self.sensing_cb_group = MutuallyExclusiveCallbackGroup()
        self.info_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Create separate callback groups for srv/client
        self.timer1_callback_group = MutuallyExclusiveCallbackGroup()
        self.service1_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the timer in the timer callback group
        self.timer1 = self.create_timer(1.0/sensing_rate, self.timer_callback, callback_group=self.timer1_callback_group)

        # Create the client in the service callback group
        self.client1 = self.create_client(DetectTags, '/rc_april_tag_detect_client/detect', callback_group=self.service1_callback_group)
        #self.create_subscription(PoseSensing, '/pose_sensing/delayed_pose', self.sensing_callback, 10, callback_group=self.sensing_cb_group)
        #self.subscription = self.create_subscription(
        #    PoseCommunication,
        #    'aruco_detection/tag_poses',
        #    self.pose_callback,
        #    qos_profiles_dict[qos_profile],#,event_callbacks=self.subscription_callbacks
        #    callback_group=self.sensing_cb_group#,event_callbacks=self.subscription_callbacks
        #)
        self.create_subscription(
            Float64MultiArray,
            '/rcvc/control_info',
            self.info_callback,
            10,
            callback_group=self.info_cb_group#,event_callbacks=self.subscription_callbacks
        )
        #self.reset_service = self.create_service(Empty, 'real_pose_real_push/robot_reset', self.reset_callback,callback_group=self.service_cb_group)
        self.start_up_service = self.create_service(Empty, 'real_pose_real_push/start_experiment', self.start_up_callback,callback_group=self.service_cb_group)
        self.set_goal_service = self.create_service(Empty, 'real_pose_real_push/set_goal', self.set_goal_callback,callback_group=self.service_cb_group)
        self.stop_service = self.create_service(Empty, 'real_pose_real_push/stop', self.stop_callback,callback_group=self.service_cb_group)
        self.settings_load_service = self.create_service(String, 'real_pose_real_push/settings_load', self.settings_load_callback,callback_group=self.service_cb_group)
        self.settings_update_service = self.create_service(Empty, 'real_pose_real_push/settings_update', self.settings_update_callback,callback_group=self.service_cb_group)



        self.cube_pose = Pose()
        self.latency_sensing = 0
        self.time_stamp_cube_origin = 0
        self.time_stamp_cube_delayed = 0
        self.time_stamp_control = 0
        self.cube_yaw = 0

        self.controller = RobotControl()
        self.cube_x_init_sensed = 0.0
        self.cube_y_init_sensed = 0.0
        self.cube_yaw_init_sensed = 0.0


        #self.cube_y_goal = cube_y_init+push_distance;

        self.back_up_counter = 0
        self.back_up_counter_limit = sensing_rate / 2
        self.step_counter = 0
        #experiment_timeout = 10
        self.control_timeout_steps = sensing_rate*experiment_timeout

        self.timeout = False
        self.terminated = False
        self.action = 0 #velocity in one direction
        self.previous_action = self.action
        #self.goal_position = 0.0
        self.distance_to_goal = 0.0
        self.max_velocity = 3.0
        self.start_time = 0.0
        #self.experiment_start_time = 0.0
        self.end_time = 0.0
        #self.reset = True
        self.counter = 0
        self.found_tags = []
        self.n_found_tags = 0
        self.tag_poses = []
        #self.tag_id = 0
        self.run_experiment = False
        #self.cube_publisher_ = self.create_publisher(Marker, 'pushed_cube', 10)
        self.tag_found = False
        self.goal_reached = False
        self.tag_not_found_counter = 0
        self.tag_not_found_limit = 10

        #q0 = 0
        #t = np.linspace(0, f_speed, n_speed_values)
        #qdf = 0
        #self.s, self.sd, _ = self.tpoly(q0, push_distance, t, v_init, qdf)

        #self.controller.reset(initial_reset=1)
        #time.sleep(2)

    def stop_callback(self,req,res):
        self.terminated = True
        self.get_logger().info(f'Experiment stopped by service!')
        return res

    def start_up_callback(self,req,res):
        self.terminated = False
        self.timeout = False
        self.goal_reached = False
        self.counter = 0
        self.cube_x_init_sensed = self.cube_pose.position.x
        self.cube_y_init_sensed = self.cube_pose.position.y
        self.cube_yaw_init_sensed = self.cube_yaw
        self.end_time = time.time()
        self.start_time = time.time()
        self.tag_not_found_counter = 0

        if goal_position == 0.0:
            self.get_logger().warn(f'Please first specify a goal position using the "real_pose_real_push/set_goal" service. It cannot be 0.0')
        else:
            self.get_logger().info(f'Starting push action with initial cube x: {self.cube_x_init_sensed}, y: {self.cube_y_init_sensed} and yaw: {self.cube_yaw_init_sensed}...')
            self.run_experiment = True

        return res

    def set_goal_callback(self,req,res):
        global goal_position, goal_x, goal_y, goal_yaw

        goal_x = self.cube_pose.position.x
        goal_y = self.cube_pose.position.y
        goal_yaw = self.cube_yaw
        goal_position = set_goal_position()

        self.get_logger().info(f'Goal position set to: {goal_position} for push direction: {push_direction}')
        general_values = [const_vel_x,const_vel_y,flip_goal_dir_sign,push_direction,qos_profile,sensing_rate,survival_time_ms,experiment_timeout,tag_id,goal_x,goal_y,goal_yaw,perform_logging]
        #overwrite config file with new goal position
        with open(config_filename, 'w') as config_file:
            config_file.write(f"[General]\n")
            for key, value in zip(general_keys,general_values):
                if key == "goal_x":
                    config_file.write(f"{key} = {goal_x}\n")
                elif key == "goal_y":
                    config_file.write(f"{key} = {goal_y}\n")
                elif key == "goal_yaw":
                    config_file.write(f"{key} = {goal_yaw}\n")
                else:
                    config_file.write(f"{key} = {value}\n")
        return res




    def settings_load_callback(self,req,res):
        filename = 'src/control_pkg/config/' + req.name
        self.get_logger().info(f'New settings from config file at: {filename}')
        set_config_params(filename)
        set_goal_position()
        res.success = 1
        self.get_logger().info(f'New experiment settings were set! {res.success}')
        return res

    def settings_update_callback(self,req,res):
        self.get_logger().info(f'Update from config file at: {config_filename}')
        set_config_params(config_filename)
        set_goal_position()
        self.get_logger().info(f'Experiment settings were updated!')
        return res

    def info_callback(self, msg):
        global time_since_last_req_ms,eef_stamp, eef_x,eef_y,eef_z,eef_roll,eef_pitch,eef_yaw
        time_since_last_req_ms = msg.data[0]
        eef_stamp = msg.data[1]
        eef_x = msg.data[2]
        eef_y = msg.data[3]
        eef_z = msg.data[4]
        eef_roll = msg.data[5]
        eef_pitch = msg.data[6]
        eef_yaw = msg.data[7]

    def timer_callback(self):

        #self.get_logger().info(f'running pose callback')
        stamp_pose_req = self.get_clock().now().nanoseconds
        if not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service /rc_april_tag_detect_client/detect not available. Trying again...')
            return
        tag = Tag()
        request = DetectTags.Request()

        tag.id = "16h5_7"
        #tag.size = 0.035

        request.tags = [tag]
        msg_size_req = asizeof(request)
        response = self.client1.call(request)
        msg_size_res = asizeof(response)
        stamp_pose_res = self.get_clock().now().nanoseconds

        tag_poses = response.tags

        self.n_found_tags = len(tag_poses)


        if self.n_found_tags == 1:
            self.tag_not_found_counter = 0
            self.cube_pose = tag_poses[0].pose.pose
            self.tag_found = True
        else:
            self.tag_found = False
        if not self.tag_found:
            self.tag_not_found_counter +=1
            self.get_logger().warn(f'Tag is not detected for {self.tag_not_found_counter} times!')

        _, _, self.cube_yaw = euler_from_quaternion(self.cube_pose.orientation)
        #self.get_logger().info(f'Counting: {self.counter}')
        #self.publish_marker()
        global execution_time, perform_logging
        if self.run_experiment:
            self.get_logger().info(f'Counter: {self.counter} / {self.control_timeout_steps}, push direction: {push_direction}, goal position: {goal_position}, current positions: x {self.cube_pose.position.x}, y {self.cube_pose.position.y}')

            if push_direction == 'x':
                #self.goal_position = self.cube_x_goal
                self.distance_to_goal = goal_position-self.cube_pose.position.x
                #abs_diff = np.abs(self.cube_pose.position.x - self.cube_x_init_sensed)
            elif push_direction == 'yaw':
                #self.goal_position = self.cube_yaw_goal
                self.distance_to_goal = goal_position-self.cube_yaw
                #abs_diff = np.abs(self.cube_yaw - self.cube_yaw_init_sensed)
            elif push_direction == 'y':
                #self.goal_position = self.cube_y_goal
                self.distance_to_goal = goal_position-self.cube_pose.position.y
                #abs_diff = np.abs(self.cube_pose.position.y - self.cube_y_init_sensed)
            else:
                self.get_logger().warn(f'Specified push direction: {push_direction} is unknown, please select x, y or yaw ')

            if flip_goal_dir_sign:
                self.distance_to_goal = -self.distance_to_goal

            #if self.counter >= self.control_timeout_steps:
            #    self.terminated = True
        #        self.get_logger().warn(f'Timeout triggered at {experiment_timeout}s!')

            if not self.timeout and (self.end_time-self.start_time) >= experiment_timeout:
                self.terminated = True
                self.get_logger().warn(f'Timeout triggered at {experiment_timeout}s!')


            if self.timeout or self.terminated:
                self.get_logger().info(f'Experiment Stopped: timeout {self.timeout}, terminated {self.terminated}')


                self.run_experiment = False
                if perform_logging and not self.terminated:
                    self.get_logger().info(f'Experiment finished, saving docs at {output_dir}...')
                    experiment_time = int(time.time())
                    output_config_filename = output_dir + str(experiment_time) + "_" + str(sensing_rate) + "_" + "_config_parameters.txt"
                    output_log_filename = output_dir + str(experiment_time) + "_" + str(sensing_rate) + "_logging.txt"

                    #header = "Episode, Execution Time, Induced latency, Time Stamp Cube Origin, Time Stamp Cube Delayed, Time Stamp EEF, cube x, cube y, cube z, cube yaw, eef x, eef y, eef yaw"
                    #comments = "Params: "
                    with open(output_log_filename, 'w') as log_file:
                        log_file.write('Counter;Execution Time;Distance to Goal;'
                           'Time Stamp Pose Requested;Time Stamp Pose Received;Time Stamp Control;Time Stamp EEF;'
                           'Action vX;Action vY;Action vZ;Action vYaw;Tag found;Cube X;Cube Y;Cube Z;Cube Yaw;'
                           'EEF X;EEF Y;EEF Z;EEF Yaw;Timeout Triggered;Time since last vel req;Message Size Request;Message Size Response\n')

                        for message in log_messages:
                            log_file.write(message + '\n')
                    new_section = "General" #push_direction + "_" + str(sensing_rate) #+ "_" + str(const_vel) + "_" + str(push_distance)+"m"
                    general_values = [const_vel_x,const_vel_y,flip_goal_dir_sign,push_direction,qos_profile,sensing_rate,survival_time_ms,experiment_timeout,tag_id,goal_x,goal_y,goal_yaw,perform_logging]
                    # Save configuration parameters to a separate file
                    with open(output_config_filename, 'w') as config_file:
                        config_file.write(f"[{new_section}]\n")
                        for key, value in zip(general_keys,general_values):
                            config_file.write(f"{key} = {value}\n")

                elif perform_logging and self.terminated:
                    self.get_logger().info(f'Experiment forced finished, saving docs at {output_dir}...')
                    experiment_time = int(time.time())
                    output_config_filename = output_dir + str(experiment_time) + "_" + str(sensing_rate) + "_" + qos_profile + "_config_parameters_failed.txt"
                    output_log_filename = output_dir + str(experiment_time) + "_" + str(sensing_rate) + "_logging_failed.txt"

                    #header = "Episode, Execution Time, Induced latency, Time Stamp Cube Origin, Time Stamp Cube Delayed, Time Stamp EEF, cube x, cube y, cube z, cube yaw, eef x, eef y, eef yaw"
                    #comments = "Params: "
                    with open(output_log_filename, 'w') as log_file:
                        log_file.write('Counter;Execution Time;Distance to Goal;'
                           'Time Stamp Pose Requested;Time Stamp Pose Received;Time Stamp Control;Time Stamp EEF;'
                           'Action vX;Action vY;Action vZ;Action vYaw;Tag found;Cube X;Cube Y;Cube Z;Cube Yaw;'
                           'EEF X;EEF Y;EEF Z;EEF Yaw;Timeout Triggered;Time since last vel req;Message Size Request;Message Size Response\n')
                        for message in log_messages:
                            log_file.write(message + '\n')


                    new_section = "General" #push_direction + "_" + str(sensing_rate) #+ "_" + str(const_vel) + "_" + str(push_distance)+"m"
                    general_values = [const_vel_x,const_vel_y,flip_goal_dir_sign,push_direction,qos_profile,sensing_rate,survival_time_ms,experiment_timeout,tag_id,goal_x,goal_y,goal_yaw,perform_logging]
                    # Save configuration parameters to a separate file
                    with open(output_config_filename, 'w') as config_file:
                        config_file.write(f"[{new_section}]\n")
                        for key, value in zip(general_keys,general_values):
                            config_file.write(f"{key} = {value}\n")
                        #for key, value in zip(keys_control,values_control):
                            #config_file.write(f"{key} = {value:.4f}\n")
                #self.get_logger().info('World reset!')
                #self.get_logger().info(f'World reset 2! initial cube pose: {self.cube_y_init_sensed}')
            elif self.distance_to_goal > 0 and not self.goal_reached:
                #self.get_logger().info(f'Pushing...')
                self.end_time = time.time()
                self.time_stamp_control = self.get_clock().now().nanoseconds

                #self.distance_to_goal = self.cube_y_goal-self.cube_pose.position.y
                self.previous_action = self.action
                #self.action = -self.get_action(self.distance_to_goal, -self.previous_action, self.max_velocity, self.max_velocity_change)
                if self.tag_not_found_counter > self.tag_not_found_limit:
                    self.action = [0.0,0.0,0.0,vqr,vqp,vqy]

                else:
                    self.action = [const_vel_x,const_vel_y,vz,vqr,vqp,vqy]#-const_vel if go_const_vel else -self.get_v_decay()
                self.controller.control(self.action)
                self.get_logger().info(f'pushing at v: {self.action}, distance to goal: {self.distance_to_goal}, time: {self.end_time}')
            else:
                self.goal_reached = True
                execution_time = self.end_time - self.start_time
                self.get_logger().info(f'goal reached after {execution_time}: distance to goal: {self.distance_to_goal}, backing up...')
                self.time_stamp_control = self.get_clock().now().nanoseconds
                #self.back_up_counter = self.back_up_counter + 1
                self.action = [-const_vel_x,-const_vel_y,-vz,-vqr,-vqp,-vqy]# if go_const_vel else v_init
                #self.action = [0.0,0.0,0.0,0.0,0.0,0.0]
                self.controller.control(self.action)
                #self.get_logger().info(f'time: {time.time()-self.end_time}')
                #self.get_logger().info('Back up!')
                if time.time()-self.end_time > execution_time:

                    self.timeout = True
                    #execution_time = self.end_time - self.start_time
                    #self.get_logger().info('Finished!')

            #history = history.append(np.array([self.current_episode, execution_time, self.latency_sensing,self.time_stamp_cube_origin, self.time_stamp_cube_delayed, time_stamp_eef, self.cube_pose.position.x, self.cube_pose.position.y, self.cube_pose.position.z,self.cube_yaw, eef_x, eef_y, eef_yaw]))
            #if not self.reset:
            log_message = (
                f"{self.counter};{execution_time:.4f};{self.distance_to_goal:.4f};{stamp_pose_req};{stamp_pose_res};{self.time_stamp_control};{time_stamp_eef};"
                f"{self.action[0]:.4f};{self.action[1]:.4f};{self.action[2]:.4f};{self.action[5]:.4f};{self.tag_found};{self.cube_pose.position.x:.4f};{self.cube_pose.position.y:.4f};{self.cube_pose.position.z:.4f};{self.cube_yaw:.4f};{eef_x:.4f};{eef_y:.4f};{eef_z:.4f};{eef_yaw:.4f};{timeout_triggered};{time_since_last_req_ms};{msg_size_req};{msg_size_res}"
            )
            log_messages.append(log_message)
            self.counter += 1


            #self.cleanup()
    def publish_marker(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = "base_link"
        marker_msg.type = Marker.CUBE
        marker_msg.pose = self.cube_pose
        marker_msg.scale.x = 0.05
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.05
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0

        self.cube_publisher_.publish(marker_msg)


    def get_action(self, distance_to_goal, previous_velocity, max_velocity, max_velocity_change):
        #
        # Calculate the distance factor as a value between 0 and 1
        distance_factor = min(1.0, distance_to_goal / push_distance)

        # Calculate the sigmoid function to smoothen the velocity increase and decrease
        sigmoid_factor = 1 / (1 + math.exp(-12 * (distance_factor - 0.5)))

        # Calculate the desired velocity at the current distance factor
        desired_velocity = max_velocity * sigmoid_factor

        # Limit the change in velocity to make the transition smooth
        #max_velocity_change = 0.1  # Tweak this value for a faster/slower transition
        velocity_change = max(-max_velocity_change, min(desired_velocity - previous_velocity, max_velocity_change))

        # Calculate and return the new velocity
        new_velocity = previous_velocity + velocity_change
        return new_velocity

    def tpoly(self,q0, qf, t, qd0=0, qdf=0):
        t0 = t
        if np.isscalar(t):
            t = np.arange(0, int(t))
        else:
            t = t.reshape(-1, 1)

        if qd0 is None:
            qd0 = 0
        if qdf is None:
            qdf = 0

        tf = np.max(t)

        X = np.array([
            [0,           0,           0,         0,       0,   1],
            [tf**5,       tf**4,       tf**3,    tf**2,    tf,  1],
            [0,           0,           0,         0,       1,   0],
            [5*tf**4,     4*tf**3,     3*tf**2,  2*tf,    1,   0],
            [0,           0,           0,         2,       0,   0],
            [20*tf**3,    12*tf**2,    6*tf,     2,       0,   0]
        ])

        coeffs = np.linalg.lstsq(X, np.array([q0, qf, qd0, qdf, 0, 0]), rcond=None)[0]

        coeffs_d = coeffs[:5] * np.array([5, 4, 3, 2, 1])
        coeffs_dd = coeffs_d[:4] * np.array([4, 3, 2, 1])

        p = np.polyval(coeffs, t0)
        pd = np.polyval(coeffs_d, t0)
        pdd = np.polyval(coeffs_dd, t0)

        return p, pd, pdd

    def get_v(self):
        current_p = push_distance - self.distance_to_goal
        #if distance_to_goal<push_distance/2:
        diff_p = np.abs(self.s-current_p)
        get_current_id = np.argmin(diff_p)
        v = self.sd[get_current_id]
        #velocity_change = max(-max_velocity_change, min(desired_velocity - previous_velocity, max_velocity_change))
        #v = previous_velocity + velocity_change
        return v

    def cleanup(self):
        self.destroy_node()
        self.controller.destroy_node()
        rclpy.shutdown()

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        random.seed()
        self.control_rate = sensing_rate #control the robot each time a new image/pose arrives
        self.control_rate_ms = float(1)/self.control_rate
        self.start = True
        self.control_cb_group = MutuallyExclusiveCallbackGroup()

        #self.cli_control = self.create_client(RobPose, '/real_ur3e_velocity_controller/set_desired_rob_pose',callback_group=self.control_cb_group)
        #self.cli_reset = self.create_client(ResetPoses, '/world_setup/reset_cubes',callback_group=self.control_cb_group)
        self.publisher_ = self.create_publisher(VelDir, '/vel_dirs', qos_profile=qos_profiles_dict[qos_profile],callback_group=self.control_cb_group)


    def control(self,a):
        msg = VelDir()
        msg.timeout_ms = survival_time_ms
        msg.goal_dir = a

        self.publisher_.publish(msg)


    def cleanup(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RealPoseRealPush()
    #node_services = ExperimentServices()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    #executor.add_node(node_services)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    executor.shutdown()


if __name__ == '__main__':
    main()
