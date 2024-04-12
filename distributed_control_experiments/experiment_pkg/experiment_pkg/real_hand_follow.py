import sys
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
    global n_prev_actions,flip_goal_dir_sign,sensing_rate,qos_profile,survival_time_ms,experiment_timeout,perform_logging,max_vel,vel_factor
    config = configparser.ConfigParser()
    config.read(filename)
    section = 'General'
    config_sec = config[section]
    #go_const_vel = config_sec.getboolean('go_const_vel', True)

    max_vel = config_sec.getfloat('max_vel', 0.01)
    vel_factor = config_sec.getint('vel_factor', 1)
    flip_goal_dir_sign = config_sec.getboolean('flip_goal_dir_sign', True)
    n_prev_actions = config_sec.getint('n_prev_actions', 10)
    #backup_ms = config_sec.getint('backup_ms', 200) #for backing up when wrist target reached
    #network = config_sec.get('network', 'private5g')
    #computation_ms = config_sec.getint('computation_ms', 0)
    #n_episode = config_sec.getint('n_episode', 5)

    sensing_rate = config_sec.getint('sensing_rate', 30)
    qos_profile = config_sec.get('quality_of_service', 'Sensor')
    push_distance = config_sec.getfloat('push_distance', 0.04)
    #n_freshness_samples = config_sec.getint('n_freshness_samples', 10)
    survival_time_ms = config_sec.getint('survival_time_ms', 200) #for low-level control
    experiment_timeout = config_sec.getint('experiment_timeout', 10)
    perform_logging = config_sec.getboolean('perform_logging', False)



keys_control = []
values_control = []

wrist_pose = Pose()
wrist_y_init = 0.58

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
config_filename = 'src/control_pkg/config/real_hand_follow.ini'
#config = read_config(config_filename)
#config = configparser.ConfigParser()
#config.read(config_filename)
#section = 'General'
#config_sec = config[section]
#go_const_vel = config_sec.getboolean('go_const_vel', True)
const_vel_x = 0.05 #config_sec.getfloat('const_vel_x', 0.05)
const_vel_y = 0.0 #config_sec.getfloat('const_vel_y', 0.00)
flip_goal_dir_sign = False #config_sec.getboolean('flip_goal_dir_sign', True)
n_prev_actions = 10
#backup_ms = 200 #config_sec.getint('backup_ms', 200) #for backing up when wrist target reached
#network = config_sec.get('network', 'private5g')
#computation_ms = config_sec.getint('computation_ms', 0)
#n_episode = config_sec.getint('n_episode', 5)
sensing_rate = 30 #config_sec.getint('sensing_rate', 30)
vel_factor = 1
max_vel = 0.2
qos_profile = 'Sensor' #config_sec.get('quality_of_service', 'Sensor')
#push_distance = config_sec.getfloat('push_distance', 0.04)
#n_freshness_samples = config_sec.getint('n_freshness_samples', 10)
survival_time_ms = 200 #config_sec.getint('survival_time_ms', 200) #for low-level control
experiment_timeout = 10 #config_sec.getint('experiment_timeout', 10)
perform_logging = False #config_sec.getboolean('perform_logging', False)


set_config_params(config_filename)
start_time = int(time.time())

general_keys = ["n_prev_actions","flip_goal_dir_sign","quality_of_service","sensing_rate","survival_time_ms","experiment_timeout","perform_logging","max_vel","vel_factor"]


time_since_last_req_ms = 0.0
eef_stamp = 0.0
eef_x = 0.0
eef_y = 0.0
eef_z = 0.0
eef_roll = 0.0
eef_pitch = 0.0
eef_yaw = 0.0
folder = ""

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

output_dir = "docs/data/logging/hand_follow/"

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

class RealHandFollow(Node):

    def __init__(self):
        super().__init__('real_hand_follow')
        self.get_logger().info(f'Starting push experiment with qos_profile: {qos_profile}')
        #random.seed()
        self.sensing_cb_group = MutuallyExclusiveCallbackGroup()
        self.info_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        #self.create_subscription(PoseSensing, '/pose_sensing/delayed_pose', self.sensing_callback, 10, callback_group=self.sensing_cb_group)
        self.subscription = self.create_subscription(
            PoseCommunication,
            'hand_detection/landmark_poses',
            self.pose_callback,
            qos_profiles_dict[qos_profile],#,event_callbacks=self.subscription_callbacks
            callback_group=self.sensing_cb_group#,event_callbacks=self.subscription_callbacks
        )
        self.create_subscription(
            Float64MultiArray,
            '/rcvc/control_info',
            self.info_callback,
            10,
            callback_group=self.info_cb_group#,event_callbacks=self.subscription_callbacks
        )
        self.start_up_service = self.create_service(Empty, 'real_hand_follow/start_experiment', self.start_up_callback,callback_group=self.service_cb_group)
        #self.set_goal_service = self.create_service(Empty, 'real_hand_follow/set_goal', self.set_goal_callback,callback_group=self.service_cb_group)
        self.stop_service = self.create_service(Empty, 'real_hand_follow/stop', self.stop_callback,callback_group=self.service_cb_group)
        self.settings_load_service = self.create_service(String, 'real_hand_follow/settings_load', self.settings_load_callback,callback_group=self.service_cb_group)
        self.settings_update_service = self.create_service(Empty, 'real_hand_follow/settings_update', self.settings_update_callback,callback_group=self.service_cb_group)

        self.wrist_pose = Pose()
        self.wrist_pose_prev = Pose()
        self.wrist_vel_x = 0.0
        self.wrist_vel_y = 0.0
        self.wrist_vel_z = 0.0
        self.latency_sensing = 0
        self.time_stamp_wrist_origin = 0
        self.time_stamp_wrist_delayed = 0
        self.time_stamp_control = 0
        self.wrist_yaw = 0

        self.controller = RobotControl()
        self.wrist_x_init_sensed = 0.0
        self.wrist_y_init_sensed = 0.0
        self.wrist_yaw_init_sensed = 0.0


        #self.wrist_y_goal = wrist_y_init+push_distance;

        self.back_up_counter = 0
        self.back_up_counter_limit = sensing_rate / 2
        self.step_counter = 0
        #experiment_timeout = 10
        self.control_timout_steps = sensing_rate*experiment_timeout

        self.timeout = False
        self.terminated = False
        self.action = []
        self.previous_vels_3D = np.zeros((n_prev_actions,3))


        #self.goal_position = 0.0
        self.distance_to_goal = 0.0
        self.max_velocity = 3.0
        self.start_time = 0.0
        #self.experiment_start_time = 0.0
        self.end_time = 0.0
        #self.reset = True
        self.counter = 0
        self.action_counter = 0
        self.found_tags = []
        self.landmark_poses = []
        #self.tag_id = 0
        self.run_experiment = False
        #self.wrist_publisher_ = self.create_publisher(Marker, 'pushed_wrist', 10)
        self.tag_found = False
        self.goal_reached = False
        self.no_pose = 1000
        self.n_no_pose = 10
        self.n_all_landmark_poses = 21
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
        self.action_counter = 0
        self.wrist_x_init_sensed = self.wrist_pose.position.x
        self.wrist_y_init_sensed = self.wrist_pose.position.y
        self.wrist_yaw_init_sensed = self.wrist_yaw
        self.end_time = time.time()
        self.start_time = time.time()
        self.get_logger().info(f'Starting hand following for {experiment_timeout}s')
        self.run_experiment = True
        global log_messages
        log_messages = []

        return res

    def settings_load_callback(self,req,res):
        filename = 'src/control_pkg/config/' + req.name
        self.get_logger().info(f'New settings from config file at: {filename}')
        set_config_params(filename)
        res.success = 1
        self.get_logger().info(f'New experiment settings were set! {res.success}')
        return res

    def settings_update_callback(self,req,res):
        self.get_logger().info(f'Update from config file at: {config_filename}')
        set_config_params(config_filename)
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

    def pose_callback(self, msg):
        msg_size_poses = asizeof(msg)
        #self.get_logger().info(f'running pose callback')
        stamp_pose_received = self.get_clock().now().nanoseconds
        #self.found_tags = np.array(msg.tag_ids)
        self.landmark_poses = msg.poses
        msg_size_image = msg.msg_size_image
        if len(self.landmark_poses) == self.n_all_landmark_poses:
            self.no_pose = 0
            self.wrist_pose_prev = self.wrist_pose
            self.wrist_pose = self.landmark_poses[0]
            self.wrist_vel_x = vel_factor*(self.wrist_pose.position.x - self.wrist_pose_prev.position.x)/sensing_rate
            self.wrist_vel_y = vel_factor*(self.wrist_pose.position.y - self.wrist_pose_prev.position.y)/sensing_rate
            self.wrist_vel_z = vel_factor*(self.wrist_pose.position.z - self.wrist_pose_prev.position.z)/sensing_rate
            if np.abs(self.wrist_vel_x) > max_vel or np.abs(self.wrist_vel_y) > max_vel or np.abs(self.wrist_vel_z) > max_vel:
                self.get_logger().warn(f'Maximum velocity {max_vel} reached!')
                if self.wrist_vel_x > max_vel: self.wrist_vel_x = max_vel
                if self.wrist_vel_x < -max_vel: self.wrist_vel_x = -max_vel
                if self.wrist_vel_y > max_vel: self.wrist_vel_y = max_vel
                if self.wrist_vel_y < -max_vel: self.wrist_vel_y = -max_vel
                if self.wrist_vel_z > max_vel: self.wrist_vel_z = max_vel
                if self.wrist_vel_z < -max_vel: self.wrist_vel_z = -max_vel
            self.previous_vels_3D[:-1,:] = self.previous_vels_3D[1:,:]
            self.previous_vels_3D[-1,:] = np.array([self.wrist_vel_x,self.wrist_vel_y,self.wrist_vel_z])
            self.action_counter += 1


        else:
            if self.no_pose > self.n_no_pose:
                self.action_counter = 0
                self.wrist_vel_x = 0.0
                self.wrist_vel_y = 0.0
                self.wrist_vel_z = 0.0


            self.no_pose += 1


        global execution_time, perform_logging
        if self.run_experiment:
            self.get_logger().info(f'Counter: {self.counter} / {self.control_timout_steps}, current vel: x {self.wrist_vel_x}, y {self.wrist_vel_y}, z {self.wrist_vel_z}')

            if self.counter >= self.control_timout_steps:
                self.terminated = True
                self.get_logger().warn(f'Timeout triggered at {experiment_timeout}s!')


            if self.timeout or self.terminated:
                self.get_logger().info(f'Experiment Stopped: timeout {self.timeout}, terminated {self.terminated}')


                self.run_experiment = False
                if perform_logging:
                    self.get_logger().info(f'Experiment finished, saving docs at {output_dir}...')
                    experiment_time = int(time.time())
                    output_config_filename = output_dir + str(experiment_time) + "_" + str(sensing_rate) + "_" + qos_profile + "_config_parameters.txt"
                    output_log_filename = output_dir + str(experiment_time) + "_" + str(sensing_rate) + "_logging.txt"

                    #header = "Episode, Execution Time, Induced latency, Time Stamp wrist Origin, Time Stamp wrist Delayed, Time Stamp EEF, wrist x, wrist y, wrist z, wrist yaw, eef x, eef y, eef yaw"
                    #comments = "Params: "
                    with open(output_log_filename, 'w') as log_file:
                        log_file.write('Counter;Execution Time;Time Stamp Image Published;'
                           'Time Stamp Image Received;Time Stamp Pose Published;Time Stamp Pose Received;Time Stamp Control;Time Stamp EEF;'
                           'Action vX;Action vY;Action vZ;Action vYaw;wrist X;wrist Y;wrist Z;wrist Yaw;'
                           'EEF X;EEF Y;EEF Z;EEF Yaw;Timeout Triggered;Time since last vel req;Message Size Image;Message Size Poses\n')

                        for message in log_messages:
                            log_file.write(message + '\n')


                    new_section = "General" #push_direction + "_" + str(sensing_rate) #+ "_" + str(const_vel) + "_" + str(push_distance)+"m"
                    general_values = [flip_goal_dir_sign,qos_profile,sensing_rate,survival_time_ms,experiment_timeout,perform_logging,max_vel,vel_factor]
                    # Save configuration parameters to a separate file
                    with open(output_config_filename, 'w') as config_file:
                        config_file.write(f"[{new_section}]\n")
                        for key, value in zip(general_keys,general_values):
                            config_file.write(f"{key} = {value}\n")
                        #for key, value in zip(keys_control,values_control):
                            #config_file.write(f"{key} = {value:.4f}\n")
                #self.get_logger().info('World reset!')
                #self.get_logger().info(f'World reset 2! initial wrist pose: {self.wrist_y_init_sensed}')
            else:
                if self.action_counter < n_prev_actions:
                    self.action = [0.0,0.0,0.0,vqr,vqp,vqy]
                    self.controller.control(self.action)
                    self.get_logger().warn(f'Not moving because not enough recent hand poses were detected')
                else:
                    median_vels_3D = np.median(self.previous_vels_3D,axis=0)
####define mapping of hand to robot
                    self.action = [median_vels_3D[2],median_vels_3D[0],-median_vels_3D[1],vqr,vqp,vqy]
                    self.controller.control(self.action)
            #history = history.append(np.array([self.current_episode, execution_time, self.latency_sensing,self.time_stamp_wrist_origin, self.time_stamp_wrist_delayed, time_stamp_eef, self.wrist_pose.position.x, self.wrist_pose.position.y, self.wrist_pose.position.z,self.wrist_yaw, eef_x, eef_y, eef_yaw]))
            #if not self.reset:
            log_message = (
                f"{self.counter};{execution_time:.4f};{msg.stamp_ns_image_published};{msg.stamp_ns_image_received};{msg.stamp_ns_pose_published};{stamp_pose_received};{self.time_stamp_control};{time_stamp_eef};"
                f"{self.action[0]:.4f};{self.action[1]:.4f};{self.action[2]:.4f};{self.action[5]:.4f};{self.wrist_pose.position.x:.4f};{self.wrist_pose.position.y:.4f};{self.wrist_pose.position.z:.4f};{self.wrist_yaw:.4f};{eef_x:.4f};{eef_y:.4f};{eef_z:.4f};{eef_yaw:.4f};{timeout_triggered};{time_since_last_req_ms};{msg_size_image};{msg_size_poses}"
            )
            log_messages.append(log_message)
            self.counter += 1


            #self.cleanup()
    def publish_marker(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = "base_link"
        marker_msg.type = Marker.CUBE
        marker_msg.pose = self.wrist_pose
        marker_msg.scale.x = 0.05
        marker_msg.scale.y = 0.05
        marker_msg.scale.z = 0.05
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0

        self.wrist_publisher_.publish(marker_msg)


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

        #self.cli_reset = self.create_client(ResetPoses, '/world_setup/reset_wrists',callback_group=self.control_cb_group)
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
    node = RealHandFollow()
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
