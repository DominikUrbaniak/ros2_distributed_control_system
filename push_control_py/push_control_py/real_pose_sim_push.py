import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import Pose#, TransformStamped
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from custom_interfaces.srv import RobPose
#from custom_interfaces.srv import SetRandomization
from custom_interfaces.srv import ResetPoses, String
from custom_interfaces.msg import PoseSensing
from custom_interfaces.srv import PoseSensingSettings
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId, PoseCommunication
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}

import time
import math
import random
import numpy as np
import configparser
import logging

keys_control = []
values_control = []

cube_pose = Pose()
cube_y_init = 0.58

v_init = 0.01
v_z_offset = 0.009#0.011
v_x_offset = -0.001
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
timeout_ms = 200 #for backing up when cube target reached

n_speed_values = 1000
config_filename = 'src/main_pkg/config/real_pose_sim_push.ini'
#config = read_config(config_filename)
config = configparser.ConfigParser()
config.read(config_filename)
section = 'General'
config_sec = config[section]
go_const_vel = config_sec.getboolean('go_const_vel', True)
const_vel = config_sec.getfloat('const_vel', 0.05)
#network = config_sec.get('network', 'private5g')
#computation_ms = config_sec.getint('computation_ms', 0)
#n_episode = config_sec.getint('n_episode', 5)
vel_decay_factor = config_sec.getfloat('vel_decay_factor', 100)
vel_decay_shift = config_sec.getfloat('vel_decay_shift', 4)
sensing_rate = config_sec.getint('sensing_rate', 30)
qos_profile = config_sec.get('quality_of_service', 'R10')
push_distance = config_sec.getfloat('push_distance', 0.04)
#n_freshness_samples = config_sec.getint('n_freshness_samples', 10)
survival_time_ms = config_sec.getint('survival_time_ms', 200) #for low-level control
experiment_timeout = config_sec.getint('experiment_timeout', 10)
perform_logging = config_sec.getboolean('perform_logging', False)
push_direction = config_sec.get('push_direction', 'y') #'x', 'y', 'yaw'
tag_id = config_sec.getint('tag_id', 0)
#start_time = int(time.time())

if len(sys.argv)>2:
    experiment_timeout = int(sys.argv[1])
    sensing_rate = int(sys.argv[2])

output_dir = "docs/data/logging/"


#network = "private5g"
#computation_ms = 0
#n_episode = 4
max_vel_change = 0.01
#f_speed = 0.5
#network = sys.argv[1]
#computation_ms = int(sys.argv[2])
#n_episode = int(sys.argv[3])
#max_vel_change = float(sys.argv[4])
#f_speed = float(sys.argv[5])  #0.1 #the lower the faster
#go_const_vel = True
#const_vel = 0.3

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



class RealPoseSimPush(Node):

    def __init__(self):
        super().__init__('real_pose_sim_push')
        self.get_logger().info(f'Starting push experiment with qos_profile: {qos_profile}')
        random.seed()
        self.sensing_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        #self.create_subscription(PoseSensing, '/pose_sensing/delayed_pose', self.sensing_callback, 10, callback_group=self.sensing_cb_group)
        self.create_subscription(
            PoseCommunication,
            'aruco_detection/tag_poses',
            self.pose_callback,
            qos_profiles[qos_profile],
            callback_group=self.sensing_cb_group#,event_callbacks=self.subscription_callbacks
        )
        self.reset_service = self.create_service(Empty, 'real_pose_sim_push/robot_reset', self.reset_callback,callback_group=self.service_cb_group)
        self.start_up_service = self.create_service(Empty, 'real_pose_sim_push/start_experiment', self.start_up_callback,callback_group=self.service_cb_group)
        self.settings_load_service = self.create_service(String, 'real_pose_sim_push/settings_load', self.settings_load_callback,callback_group=self.service_cb_group)
        self.settings_update_service = self.create_service(Empty, 'real_pose_sim_push/settings_update', self.settings_update_callback,callback_group=self.service_cb_group)

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

        self.cube_y_goal = cube_y_init+push_distance;

        self.back_up_counter = 0
        self.back_up_counter_limit = sensing_rate / 2
        self.step_counter = 0
        #experiment_timeout = 10
        self.control_timout_steps = sensing_rate*experiment_timeout

        self.timeout = False
        self.action = 0 #velocity in one direction
        self.previous_action = self.action
        self.distance_to_goal = push_distance
        self.max_velocity = 3.0
        self.max_velocity_change = max_vel_change
        self.start_time = 0.0
        self.end_time = 0.0
        self.reset = True
        self.counter = 0
        self.found_tags = []
        self.tag_poses = []
        #self.tag_id = 0
        self.run_experiment = False
        #self.cube_publisher_ = self.create_publisher(Marker, 'pushed_cube', 10)
        self.tag_found = False
        #q0 = 0
        #t = np.linspace(0, f_speed, n_speed_values)
        #qdf = 0
        #self.s, self.sd, _ = self.tpoly(q0, push_distance, t, v_init, qdf)

        self.controller.reset(initial_reset=1)
        time.sleep(2)

    def reset_callback(self,req,res):
        #id = req.objid
        self.get_logger().info(f'Resetting experiment...')
        self.controller.reset()
        self.reset = True
        self.counter = 0
        #res.success = True
        return res

    def start_up_callback(self,req,res):
        self.cube_x_init_sensed = self.cube_pose.position.x
        self.cube_y_init_sensed = self.cube_pose.position.y
        self.cube_yaw_init_sensed = self.cube_yaw
        self.end_time = time.time()
        if self.reset:
            self.get_logger().info(f'Starting push action with initial cube x: {self.cube_x_init_sensed}, y: {self.cube_y_init_sensed} and yaw: {self.cube_yaw_init_sensed}...')
            self.run_experiment = True
            #start_time = int(time.time())
            #output_log_filename = output_dir + str(start_time) + "_" + push_direction + "_" + str(sensing_rate) + "_logging.txt"
        else:
            self.get_logger().warn(f'Please reset the experiment first using the reset service!')
        return res

    def set_config_params(self,filename):
        global go_const_vel,const_vel,vel_decay_factor,vel_decay_shift,sensing_rate,qos_profile,push_distance,survival_time_ms,experiment_timeout,perform_logging,push_direction,tag_id
        config = configparser.ConfigParser()
        config.read(filename)
        section = 'General'
        config_sec = config[section]
        go_const_vel = config_sec.getboolean('go_const_vel', True)
        const_vel = config_sec.getfloat('const_vel', 0.05)
        #network = config_sec.get('network', 'private5g')
        #computation_ms = config_sec.getint('computation_ms', 0)
        #n_episode = config_sec.getint('n_episode', 5)
        vel_decay_factor = config_sec.getfloat('vel_decay_factor', 100)
        vel_decay_shift = config_sec.getfloat('vel_decay_shift', 4)
        sensing_rate = config_sec.getint('sensing_rate', 30)
        qos_profile = config_sec.get('quality_of_service', 'R10')
        push_distance = config_sec.getfloat('push_distance', 0.04)
        #n_freshness_samples = config_sec.getint('n_freshness_samples', 10)
        survival_time_ms = config_sec.getint('survival_time_ms', 200) #for low-level control
        experiment_timeout = config_sec.getint('experiment_timeout', 10)
        perform_logging = config_sec.getboolean('perform_logging', False)
        push_direction = config_sec.get('push_direction', 'y') #'x', 'y', 'yaw'
        tag_id = config_sec.getint('tag_id', 0)


    def settings_load_callback(self,req,res):
        filename = 'src/main_pkg/config/' + req.name
        self.get_logger().warn(f'New settings from config file at: {filename}')
        self.set_config_params(filename)
        res.success = 1
        self.get_logger().warn(f'New experiment settings were set! {res.success}')
        return res

    def settings_update_callback(self,req,res):
        self.get_logger().warn(f'Update from config file at: {config_filename}')
        self.set_config_params(config_filename)
        self.get_logger().warn(f'Experiment settings were updated!')
        return res

    #def _timer_cb(self):
    def pose_callback(self, msg):
        #self.get_logger().info(f'running pose callback')
        stamp_pose_received = self.get_clock().now().nanoseconds
        self.found_tags = np.array(msg.tag_ids)
        self.tag_poses = msg.poses
        if len(self.found_tags) > 0:

            tag_index = np.where(self.found_tags == tag_id)[0]
            #self.get_logger().info(f'Detected tags: {self.found_tags},{self.found_tags[0]}, tag_index: {tag_index}')
            if tag_index.size > 0:
                self.cube_pose = self.tag_poses[tag_index[0]]
                self.tag_found = True
            else:
                self.tag_found = False
        else:
            self.tag_found = False
        if not self.tag_found:
            self.get_logger().warn(f'Tag is not detected!')

        _, _, self.cube_yaw = euler_from_quaternion(self.cube_pose.orientation)
        #self.get_logger().info(f'Counting: {self.counter}')
        #self.publish_marker()
        global execution_time, accuracy
        if self.run_experiment:
            self.get_logger().info(f'Counter: {self.counter} / {self.control_timout_steps}, push direction: {push_direction}')
            abs_diff = 1000.0
            if push_direction == 'x':
                abs_diff = np.abs(self.cube_pose.position.x - self.cube_x_init_sensed)
            elif push_direction == 'yaw':
                abs_diff = np.abs(self.cube_yaw - self.cube_yaw_init_sensed)
            elif push_direction == 'y':
                abs_diff = np.abs(self.cube_pose.position.y - self.cube_y_init_sensed)
            else:
                self.get_logger().warn(f'Specified push direction: {push_direction} is unknown, please select x, y or yaw ')

            if self.counter >= self.control_timout_steps:
                self.timeout = True
                self.get_logger().warn(f'Timeout triggered!')

            if self.timeout:
                self.get_logger().info(f'Experiment Stopped: {self.timeout}')

                self.step_counter = 0
                self.action = 0
                self.previous_action = self.action
                self.timeout = False
                self.reset = False
                self.run_experiment = False
                if perform_logging:
                    self.get_logger().info(f'Experiment finished, saving docs at {output_dir}...')
                    experiment_time = int(time.time())
                    output_config_filename = output_dir + str(experiment_time) + "_" + push_direction + "_" + str(sensing_rate) + "_config_parameters.txt"
                    output_log_filename = output_dir + str(experiment_time) + "_" + push_direction + "_" + str(sensing_rate) + "_logging.txt"

                    #header = "Episode, Execution Time, Induced latency, Time Stamp Cube Origin, Time Stamp Cube Delayed, Time Stamp EEF, cube x, cube y, cube z, cube yaw, eef x, eef y, eef yaw"
                    #comments = "Params: "
                    with open(output_log_filename, 'w') as log_file:
                        log_file.write('Counter;Execution Time;Absolute Difference;Time Stamp Image Published;'
                           'Time Stamp Image Received; Time Stamp Pose Published; Time Stamp Pose Received;Time Stamp Control;Time Stamp EEF;Action;Cube X;Cube Y;Cube Z;Cube Yaw'
                           'EEF X;EEF Y;EEF Yaw;Timeout Triggered;Time since last vel req\n')

                        for message in log_messages:
                            log_file.write(message + '\n')

                    general_keys = ["go_const_vel","const_vel","vel_decay_factor","vel_decay_shift","quality_of_service","sensing_rate","push_distance","survival_time_ms","experiment_timeout","push_direction","tag_id"]
                    general_values = [go_const_vel,const_vel,vel_decay_factor, vel_decay_shift,qos_profile,sensing_rate,push_distance,survival_time_ms,experiment_timeout,push_direction,tag_id]
                    new_section = push_direction + "_" + str(sensing_rate) #+ "_" + str(const_vel) + "_" + str(push_distance)+"m"

                    # Save configuration parameters to a separate file
                    with open(output_config_filename, 'w') as config_file:
                        config_file.write(f"[{new_section}]\n")
                        for key, value in zip(general_keys,general_values):
                            config_file.write(f"{key} = {value}\n")
                        for key, value in zip(keys_control,values_control):
                            config_file.write(f"{key} = {value:.4f}\n")
                #self.get_logger().info('World reset!')
                #self.get_logger().info(f'World reset 2! initial cube pose: {self.cube_y_init_sensed}')
            elif abs_diff < 0.001: #in simulation thius should always be the same, in real world add small deviations
                #self.get_logger().info(f'Approaching...')
                self.time_stamp_control = self.get_clock().now().nanoseconds
                self.action = -const_vel# if go_const_vel else -v_init
                self.controller.control(self.action)
                self.start_time = time.time()
                self.get_logger().info(f'Approaching! Time: {self.start_time}, current push distance: {self.cube_pose.position.y - self.cube_y_init_sensed}')
            elif abs_diff < push_distance:
                #self.get_logger().info(f'Pushing...')
                self.end_time = time.time()
                self.time_stamp_control = self.get_clock().now().nanoseconds

                self.distance_to_goal = self.cube_y_goal-self.cube_pose.position.y
                self.previous_action = self.action
                #self.action = -self.get_action(self.distance_to_goal, -self.previous_action, self.max_velocity, self.max_velocity_change)
                self.action = -const_vel if go_const_vel else -self.get_v_decay()
                self.controller.control(self.action)
                self.get_logger().info(f'pushing at v: {self.action}, current push distance: {self.cube_pose.position.y - self.cube_y_init_sensed}, time: {self.end_time}, , current cube pose: {self.cube_pose.position.y}')
            else:
                self.get_logger().info(f'distance pushed: {np.abs(self.cube_pose.position.y - self.cube_y_init_sensed)}, backing up...')
                self.time_stamp_control = self.get_clock().now().nanoseconds
                #self.back_up_counter = self.back_up_counter + 1
                self.action = const_vel# if go_const_vel else v_init
                self.controller.control(self.action)
                #self.get_logger().info(f'time: {time.time()-self.end_time}')
                #self.get_logger().info('Back up!')
                if 1000*(time.time()-self.end_time) > timeout_ms:

                    self.timeout = True
                    execution_time = self.end_time - self.start_time
                    accuracy = self.cube_pose.position.y - self.cube_y_goal
                    #self.get_logger().info('Finished!')



            #history = history.append(np.array([self.current_episode, execution_time, self.latency_sensing,self.time_stamp_cube_origin, self.time_stamp_cube_delayed, time_stamp_eef, self.cube_pose.position.x, self.cube_pose.position.y, self.cube_pose.position.z,self.cube_yaw, eef_x, eef_y, eef_yaw]))
            #if not self.reset:
            log_message = (
                f"{self.counter};{execution_time:.4f};{abs_diff:.4f};{msg.stamp_ns_image_published};{msg.stamp_ns_image_received};{msg.stamp_ns_pose_published};{stamp_pose_received};{self.time_stamp_control};{time_stamp_eef};"
                f"{-self.action:.4f};{self.cube_pose.position.x:.4f};{self.cube_pose.position.y:.4f};{self.cube_pose.position.z:.4f};{self.cube_yaw:.4f};{eef_x:.4f};{eef_y:.4f};{eef_yaw:.4f};{timeout_triggered};{time_since_last_req_ms}"
            )
            log_messages.append(log_message)
            self.counter += 1
            #self.step_counter = self.step_counter + 1 #compare step here with the one in a parallel thread (if threads run at different rates)

                    #config_file.write(f"timeout_ms = {value}\n")

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
    def get_v_decay(self):
        v = np.min([const_vel, np.exp(vel_decay_factor*self.distance_to_goal-vel_decay_shift)])
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
        #self.control2_cb_group = MutuallyExclusiveCallbackGroup()
        self.sensing_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli_control = self.create_client(RobPose, '/velocity_controller/set_desired_rob_pose',callback_group=self.control_cb_group)
        self.cli_reset = self.create_client(ResetPoses, '/world_setup/reset_cubes',callback_group=self.control_cb_group)

        self.x_offset = -0.027
        self.pose_eef_init = Pose()
        self.pose_cube_init = Pose()
        self.pose_eef_init.position.x = 0.1 + self.x_offset
        self.pose_eef_init.position.y = -0.49 #-0.39#,-0.49
        self.pose_eef_init.position.z = 0.13 #0.08#,0.112
        self.eef_init_roll = 0.0 #0.8
        self.eef_init_pitch = np.pi
        self.eef_init_yaw = 0.0
        self.pose_eef_init.orientation = euler_to_quaternion(self.eef_init_roll,self.eef_init_pitch,self.eef_init_yaw)#,euler_to_quaternion(0,np.pi,0)
        self.pose_cube_init.position.x = -0.10
        self.pose_cube_init.position.y = cube_y_init
        self.pose_cube_init.position.z = 0.025
        self.cube_init_roll = 0.0
        self.cube_init_pitch = np.pi
        self.cube_init_yaw = 0.0
        self.pose_cube_init.orientation = euler_to_quaternion(self.cube_init_roll,self.cube_init_pitch,self.cube_init_yaw)
        self.pose_gripper = 0.6
        #self.obj_id = 5
        #self.n_freshness_samples = 10
        #self.cli_sensing = self.create_client(PoseSensing, '/pose_sensing/get_delayed_pose',callback_group=self.sensing_cb_group)

        '''self.cli_sensing_settings = self.create_client(PoseSensingSettings, '/pose_sensing/set_settings')
        while not self.cli_sensing_settings.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PoseSensingSettings.Request()
        self.req.objid = self.obj_id
        self.req.computation_ms = computation_ms
        self.req.network = network
        self.req.n_freshness_samples = n_freshness_samples
        self.req.sensing_rate = sensing_rate
        self.req.qos_profile = qos_profile
        #self.eef_pose = TransformStamped()

        self.future = self.cli_sensing_settings.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'pose settings success: {self.future.result().success}, control rate: {self.control_rate_ms}, delay mode: {self.future.result().delay_mode}')
'''
        global keys_control, values_control
        keys_control = ["cube_init_x","cube_init_y","cube_init_z","cube_init_roll","cube_init_pitch","cube_init_yaw","eef_init_x","eef_init_y","eef_init_z","eef_init_roll","eef_init_pitch","eef_init_yaw","pose_gripper","v_x_offset","v_z_offset","v_init","timeout_ms"]
        values_control = [self.pose_cube_init.position.x,self.pose_cube_init.position.y,self.pose_cube_init.position.z,self.cube_init_roll,self.cube_init_pitch,self.cube_init_yaw,self.pose_eef_init.position.x,self.pose_eef_init.position.y,self.pose_eef_init.position.z,self.eef_init_roll,self.eef_init_pitch,self.eef_init_yaw,self.pose_gripper,v_x_offset,v_z_offset,v_init,timeout_ms]

    def reset(self,initial_reset = 0):
        while not self.cli_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for control service...')
        request = RobPose.Request()
        request.timeout_ms = 1000
        request.cart_pose = 1

        request.goal_dir = [self.pose_eef_init.position.x,self.pose_eef_init.position.y,self.pose_eef_init.position.z,self.eef_init_roll,self.eef_init_pitch,self.eef_init_yaw,self.pose_gripper]
        #self.get_logger().info(f'Sending to controller: {a}')
        future = self.cli_control.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'resetting robot...')
        else:
            self.get_logger().warning('Failed to control')

        '''while not self.cli_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset service...')
        request = ResetPoses.Request()
        request.pose_cube = self.pose_cube_init
        future = self.cli_reset.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'World is successfully reset: {future.result().success}')
        else:
            self.get_logger().warning('Failed to reset')'''


    def control(self,a):
        while not self.cli_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for control service...')
        #self.get_logger().info('Connected to control service!')
        request = RobPose.Request()
        request.timeout_ms = survival_time_ms
        request.cart_pose = 0

        request.goal_dir = [v_x_offset,a,v_z_offset,0.0,0.0,0.0,self.pose_gripper]
        #self.get_logger().info(f'Sending to controller: {a}')
        future = self.cli_control.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            #self.get_logger().info('request successful!')
            global time_stamp_eef, manipulability_index, eef_x, eef_y, eef_yaw, timeout_triggered, time_since_last_req_ms
            time_stamp_eef = future.result().time_stamp
            manipulability_index = future.result().manipulability_index
            eef_x = future.result().eef_x
            eef_y = future.result().eef_y
            eef_yaw = future.result().eef_yaw
            timeout_triggered = future.result().timeout_triggered
            time_since_last_req_ms = future.result().time_since_last_req_ms
            #self.get_logger().info(f'response received')
        else:
            self.get_logger().warning('Failed to control')



    def cleanup(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RealPoseSimPush()
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
