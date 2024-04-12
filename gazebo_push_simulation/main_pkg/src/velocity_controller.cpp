// node that controls the joint velocities to keep a joint configuration
// subscribe to joint_poses
// publish to velocity controller

// This program controls the UR robot arm to push an object using a quadratic interpolation
// to generate joint velocities based on the deviation between the desired and actual end-effector position

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "custom_interfaces/srv/rob_conf.hpp"

#include <iostream>
#include <fstream>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

// Define main program parameters

int control_rate = 500;                                 // frequency of control loop
int control_rate_ms = (float)1/control_rate * 1000;

// ROS node parameters
int rate = 1000;                                        // rate of ROS node
bool activate_stabilization = true;
bool got_initial_config = false;
bool logging = false;                                   // flag to enable logging
int counter = 0;                                        //counting the steps backwards after cube reached the goal

// Joint parameters
int n_joints = 12;                                      // number of UR joints plus gripper joint
int n_passive_joints = 5;                               // number of passive gripper joints
double vel_max = M_PI;                                  // maximum joint velocity
double vel;                                             // joint velocity
std::vector<double> vels;                               // vector of joint velocities
std::vector<std::vector<double>> history;               // vector of historical joint configurations

// Scaling factors for motion planning
double scaler_cubic = 50000;                            // scaling factor for cubic interpolation
int scaler_square = 10000;                              // scaling factor for square function
double scaler_lin = 400;                                // scaling factor for linear function

// Time parameters
std::vector<double> time_stamps;                        // vector of time stamps
bool get_time_stamp = false;                            // flag to get time stamp
double time_pose_received;                              // time that end-effector pose was received
rclcpp::Clock clk;                                      // ROS clock
rclcpp::Time stamp;                                     // time stamp for current state of robot arm
rclcpp::Time eef_vc_stamp;                              // time stamp for receiving end-effector pose
int secs;                                               // number of seconds for time stamp
bool trigger = false;                                   // flag to trigger moment when cube reached its goal pose

// End-effector parameters
double difference;                                      // difference between desired and actual end-effector pose
std::vector<double> tcp_position;                       // TCP position
std::string reference_frame = "base_link";              // reference frame for transform
std::vector<double> ik_offsets = {0.025,-0.002,0.055};  // offsets for inverse kinematics
std::vector<double> multipliers = {-1,-1,1};            // multipliers for inverse kinematics
geometry_msgs::msg::TransformStamped transform_ur_eef;  // transform from UR base to end-effector frame

// Logging parameters
std::ofstream myfile;                                   // file for logging data
std::string filename;                                   // name of log file
bool save_params_to_csv = false;                        // flag to save parameters to CSV file
std::string dir_path = "";   // path to save CSV file

std::map<std::string, int> map_joints_to_joint_states_id = { // map joint names to joint states IDs
  { "shoulder_pan_joint", 0 },
  { "shoulder_lift_joint", 1 },
  { "elbow_joint", 8 },
  { "wrist_1_joint", 2 },
  { "wrist_2_joint", 3 },
  { "wrist_3_joint", 4 },
  { "gripper_right_driver_joint", 5 },
  { "gripper_left_driver_joint", 7 },
  { "gripper_right_spring_link_joint", 6 },
  { "gripper_left_spring_link_joint", 11 },
  { "gripper_right_follower_joint", 9 },
  { "gripper_left_follower_joint", 10 }
};

std::vector<double> current_joint_configuration(n_joints,0.0);
std::vector<double> desired_joint_configuration(n_joints,0.0);
std::vector<double> diffs(n_joints,0.0);
//std::vector<double> set_joint_velocities(n_joints,0.0);
double dev2vel2(double deviation){
  double vel;
  if(deviation<0.0){
    vel = -scaler_square*pow(deviation,2);//std::exp()
  }
  else {
    vel = scaler_square*pow(deviation,2);
  }

  if(vel > vel_max){vel = vel_max;}
  else if(vel < -vel_max){vel = -vel_max;}
  return vel;
}

// Define VelocityPublisher class, derived from rclcpp::Node
class VelocityPublisher : public rclcpp::Node
{
public:
  // Constructor
  VelocityPublisher() : Node("velocity_publisher")
  {
    // Subscribe to joint state topic and set callback function
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rate, std::bind(&VelocityPublisher::topic_callback, this, _1));

    // Create publisher for velocity commands
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller_all/commands", control_rate);

    // Create timer for sending velocity commands
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_rate_ms), std::bind(&VelocityPublisher::timer_callback, this));

    // Create service for setting desired joint configuration
    service_ = this->create_service<custom_interfaces::srv::RobConf>(
      "/velocity_controller/set_desired_joint_config",
      std::bind(&VelocityPublisher::set_desired_joint_config, this, _1, _2));

    // Create buffer and listener for TF2 transformations
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  // Define callback function for joint state topic subscription
  void topic_callback(const sensor_msgs::msg::JointState &msg) const {
    // Map the joint names to their indices in the message
    current_joint_configuration[0] = msg.position[map_joints_to_joint_states_id["shoulder_pan_joint"]];
    current_joint_configuration[1] = msg.position[map_joints_to_joint_states_id["shoulder_lift_joint"]];
    current_joint_configuration[2] = msg.position[map_joints_to_joint_states_id["elbow_joint"]];
    current_joint_configuration[3] = msg.position[map_joints_to_joint_states_id["wrist_1_joint"]];
    current_joint_configuration[4] = msg.position[map_joints_to_joint_states_id["wrist_2_joint"]];
    current_joint_configuration[5] = msg.position[map_joints_to_joint_states_id["wrist_3_joint"]];
    current_joint_configuration[6] = msg.position[map_joints_to_joint_states_id["gripper_right_driver_joint"]];
    current_joint_configuration[7] = msg.position[map_joints_to_joint_states_id["gripper_left_driver_joint"]];
    current_joint_configuration[8] = msg.position[map_joints_to_joint_states_id["gripper_right_spring_link_joint"]];
    current_joint_configuration[9] = msg.position[map_joints_to_joint_states_id["gripper_left_spring_link_joint"]];
    current_joint_configuration[10] = msg.position[map_joints_to_joint_states_id["gripper_right_follower_joint"]];
    current_joint_configuration[11] = msg.position[map_joints_to_joint_states_id["gripper_left_follower_joint"]];

    try {
      // Get the transformation between the reference frame and the end effector frame
      transform_ur_eef = tf_buffer_->lookupTransform(reference_frame, "wrist_3_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      // If the transformation lookup fails, log an error and return
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", reference_frame.c_str(), "wrist_3_link", ex.what());
      return;
    }

    // Record the timestamp when the end effector position was last updated
    eef_vc_stamp = clk.now();

    // Compute the TCP position using the end effector position and the multipliers and offsets
    tcp_position = {
      (transform_ur_eef.transform.translation.x * multipliers[0]) - ik_offsets[0],
      (transform_ur_eef.transform.translation.y * multipliers[1]) - ik_offsets[1],
      (transform_ur_eef.transform.translation.z * multipliers[2]) - ik_offsets[2]
    };
  }
  void set_desired_joint_config(
  std::shared_ptr<custom_interfaces::srv::RobConf::Request> req,
  std::shared_ptr<custom_interfaces::srv::RobConf::Response> res
  )
  {
    // Update desired joint configuration
    for (int i = 0; i < n_joints; i++) {
      if (i < 7) {
        desired_joint_configuration[i] = req->conf[i];
      }
      else if (i >= 10) {
        desired_joint_configuration[i] = -req->conf[6];
      }
      else {
        desired_joint_configuration[i] = req->conf[6];
      }
    }

    // Update time stamps
    time_stamps = req->time_stamps;

    // Start logging
    if (req->start) {
      save_params_to_csv = true;
      filename = dir_path + std::to_string((int)time_stamps[1]) + "_times_" + std::to_string(scaler_square) + "vc_" + std::to_string(control_rate_ms) + "ms.csv";
      myfile.open(filename);
      myfile << "trigger,secs_vc,secs_eef_vc,secs_eef,secs_sensed,secs_delayed,secs_ik,eef_vc_x,eef_vc_y,eef_vc_z,vels[0],vels[1],vels[2],vels[3],vels[4],vels[5],desired_conf[0],desired_conf[1],desired_conf[2],desired_conf[3],desired_conf[4],desired_conf[5],current_conf[0],current_conf[1],current_conf[2],current_conf[3],current_conf[4],current_conf[5]\n";
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "started logging...");
    }

    // Update trigger
    trigger = req->event_trigger;
    if (req->event_trigger) {
      get_time_stamp = true;
    }

    // Save history to CSV
    if (save_params_to_csv) {
      secs = time_stamps[1];
      history.push_back({
        (float)trigger,
        stamp.seconds() - secs,
        eef_vc_stamp.seconds() - secs,
        time_stamps[2] - secs,
        time_stamps[3] - secs,
        time_stamps[4] - secs,
        time_stamps[5],
        tcp_position[0],
        tcp_position[1],
        tcp_position[2],
        vels[0],
        vels[1],
        vels[2],
        vels[3],
        vels[4],
        vels[5],
        desired_joint_configuration[0],
        desired_joint_configuration[1],
        desired_joint_configuration[2],
        desired_joint_configuration[3],
        desired_joint_configuration[4],
        desired_joint_configuration[5],
        current_joint_configuration[0],
        current_joint_configuration[1],
        current_joint_configuration[2],
        current_joint_configuration[3],
        current_joint_configuration[4],
        current_joint_configuration[5]
      });
    }

  // Set response success
  res->success = true;

  }

  void timer_callback()
  {
    // Declare variables and initialize if necessary
    auto message = std_msgs::msg::Float64MultiArray();
    std::vector<float> diffs(n_joints);
    std::vector<float> vels(n_joints);

    // Handle initial configuration
    if (!got_initial_config)
    {
      desired_joint_configuration = current_joint_configuration;
      got_initial_config = true;
      RCLCPP_INFO(this->get_logger(), "Saving as desired configuration: '(%f, %f, %f, %f, %f, %f, %f)'",
                  desired_joint_configuration[0], desired_joint_configuration[1], desired_joint_configuration[2],
                  desired_joint_configuration[3], desired_joint_configuration[4], desired_joint_configuration[5],
                  desired_joint_configuration[6]);
    }

    // Calculate velocity for each joint
    for (int i = 0; i < n_joints; i++)
    {
      diffs[i] = desired_joint_configuration[i] - current_joint_configuration[i];
      float vel = dev2vel2(diffs[i]);
      vels[i] = vel;
      message.data.push_back(vel);
    }

    // Publish velocity message and save data to history
    publisher_->publish(message);
    if (get_time_stamp)
    {
      if (counter >= 30)
      {
        if (save_params_to_csv)
        {
          // Save history to CSV file
          for (const auto& row : history)
          {
            for (const auto& item : row)
            {
              myfile << item << ",";
            }
            myfile << "\n";
          }
          myfile.close();
          save_params_to_csv = false;
          RCLCPP_INFO(this->get_logger(), "Finished writing times to file to %s: %d, history size: %d",
                      filename.c_str(), control_rate_ms, static_cast<int>(history.size()));
        }

        // Clear history
        counter = 0;
        history.clear();
      }

      // Save data to history
      float secs = time_stamps[1];
      history.push_back({static_cast<float>(trigger), stamp.seconds() - secs, eef_vc_stamp.seconds() - secs,
                          time_stamps[2] - secs, time_stamps[3] - secs, time_stamps[4] - secs, time_stamps[5],
                          tcp_position[0], tcp_position[1], tcp_position[2], vels[0], vels[1], vels[2], vels[3], vels[4], vels[5],
                          desired_joint_configuration[0], desired_joint_configuration[1], desired_joint_configuration[2],
                          desired_joint_configuration[3], desired_joint_configuration[4], desired_joint_configuration[5],
                          current_joint_configuration[0], current_joint_configuration[1], current_joint_configuration[2],
                          current_joint_configuration[3], current_joint_configuration[4], current_joint_configuration[5]});
      get_time_stamp = false;
      counter++;
    }
  }

  // Declare class members
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Service<custom_interfaces::srv::RobConf>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());

  rclcpp::shutdown();
  return 0;
}
