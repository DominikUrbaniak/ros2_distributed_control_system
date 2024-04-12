
// Include necessary libraries
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include "custom_interfaces/srv/rob_conf.hpp"
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <kinenik/kinenik_ur.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <gazebo_msgs/srv/get_entity_state.hpp>

// Set namespace
using namespace std::chrono_literals;

// General parameters
int n_runs = 4;
std::vector<double> goal_states;
double average = 0;
int latency_ms = 100;
std::ofstream myfile;
std::string filename;
double cube_y_init = 0.65;
double cube_x_init = -0.1;
double cube_dim = 0.025;
int initial_cube_id = 3;
double x_offset = -0.027;
double cube_offset = 0.003;
int vc_scaler = 10000;
double tcp_x = 0.1 +x_offset;
double tcp_qp = M_PI;
double tcp_qy = 0;
double tcp_gripper = 0.2;
double tcp_y_init = -0.39;
double tcp_y = tcp_y_init;
double tcp_z = 0.08;
double tcp_qr = 0.8;
double push_distance = 0.06;
double cube_y_goal = cube_y_init+push_distance;
std::string fromFrameRel = "base_link";
std::string toFrameRel = "wrist_3_link";
int set_cubes_after_number = 10;
int counter = 0;
double step_size = 0.001;

// Set up robot and time variables
rclcpp::Clock clk;
rclcpp::Time time_pose_sensed;
rclcpp::Time time_pose_delayed;
rclcpp::Time time_eef;
rclcpp::Time time_pose_at_finished_pushing;
KinenikUR myrobot("UR5e");

// Initialize pose and ID variables
geometry_msgs::msg::Pose get_model_state_pose;
geometry_msgs::msg::Pose get_model_state_pose_delayed;
int get_eef_y;
bool gazebo_model_id_set = false;
int gazebo_set_state_counter = 0;

class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    std::string frame_id;
    std::string name;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Pose pose_init;
public:
    inline unsigned int getTagID() {return id;}
    inline unsigned int getGazeboModelID() {return id;}
    inline std::string getname() {return name;}
    inline double getx() {return pose.position.x;}
    inline double gety() {return pose.position.y;}
    inline double getz() {return pose.position.z;}
    inline double getqx() {return pose.orientation.x;}
    inline double getqy() {return pose.orientation.y;}
    inline double getqz() {return pose.orientation.z;}
    inline double getqw() {return pose.orientation.w;}
    inline geometry_msgs::msg::Pose getPose() {return pose;}
    inline double getx_init() {return pose_init.position.x;}
    inline double gety_init() {return pose_init.position.y;}
    inline double getz_init() {return pose_init.position.z;}
    inline double getqx_init() {return pose_init.orientation.x;}
    inline double getqy_init() {return pose_init.orientation.y;}
    inline double getqz_init() {return pose_init.orientation.z;}
    inline double getqw_init() {return pose_init.orientation.w;}
    inline geometry_msgs::msg::Pose getPose_init() {return pose_init;}

    inline void setTagID(unsigned int i) {id=i;}
    inline void setGazeboModelID(unsigned int i) {id=i;}
    inline void setname(std::string s) {name=s;}
    inline void setPose(geometry_msgs::msg::Pose g) {pose=g;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline void setPose_init(geometry_msgs::msg::Pose g) {pose_init=g;}
    inline void setx_init(double v) {pose_init.position.x=v;}
    inline void sety_init(double v) {pose_init.position.y=v;}
    inline void setz_init(double v) {pose_init.position.z=v;}
    inline void setqx_init(double v) {pose_init.orientation.x=v;}
    inline void setqy_init(double v) {pose_init.orientation.y=v;}
    inline void setqz_init(double v) {pose_init.orientation.z=v;}
    inline void setqw_init(double v) {pose_init.orientation.w=v;}

};

std::vector<ObjectInfo> objects;

void SetWorld()
{
    ObjectInfo obj;

    objects.clear();


    obj.setTagID(0);
    obj.setGazeboModelID(0);
    obj.setname("cube_tag0_grey");
    obj.setx(cube_x_init);
    obj.sety(3);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());

    objects.push_back(obj);

    obj.setTagID(1);
    obj.setGazeboModelID(1);
    obj.setname("cube_tag1_violet");
    obj.setx(cube_x_init+cube_dim+cube_offset);
    obj.sety(3);
    obj.setz(0.025+0.01);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(2);
    obj.setGazeboModelID(2);
    obj.setname("cube_tag2_indigo");
    obj.setx(cube_x_init-cube_dim-cube_offset);
    obj.sety(3);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(3);
    obj.setGazeboModelID(3);
    obj.setname("cube_tag3_blue");
    obj.setx(cube_x_init);
    obj.sety(cube_y_init);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(4);
    obj.setGazeboModelID(4);
    obj.setname("cube_tag4_green");
    obj.setx(cube_x_init-cube_dim-cube_offset);
    obj.sety(3);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(5);
    obj.setGazeboModelID(5);
    obj.setname("cube_tag5_yellow");
    obj.setx(cube_x_init+cube_dim+cube_offset);
    obj.sety(3);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(6);
    obj.setGazeboModelID(6);
    obj.setname("cube_tag6_orange");
    obj.setx(1);
    obj.sety(3);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(7);
    obj.setGazeboModelID(7);
    obj.setname("cube_tag7_red");
    obj.setx(1.1);
    obj.sety(3);
    obj.setz(0.025);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);
}

namespace push_translation_gazebo
{
  // Class for managing a Gazebo simulation node.
  class GazeboNode : public rclcpp::Node
  {
  public:
    // Constructor.
    GazeboNode() : Node("gazebo_node")
    {
      // Set the world state to the initial configuration.
      SetWorld();

      // Create callback groups for timer and client.
      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // Create a timer to send requests to Gazebo.
      timer_ptr_ = this->create_wall_timer(100ms, std::bind(&GazeboNode::timer_callback, this), timer_cb_group_);

      // Create a client to send requests to Gazebo.
      client_ptr_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
        "/gazebo_state/set_entity_state",
        rmw_qos_profile_services_default,
        client_cb_group_);

      // Wait until the Gazebo service is available.
      while (!client_ptr_->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
      }
    }

  private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_ptr_;
    int gazebo_set_state_counter_{0};
    int counter_{0};

    // Timer callback function for updating the Gazebo world.
    void timer_callback()
    {
      // Wait until a certain number of iterations have been reached before setting objects.
      if (counter_ >= set_cubes_after_number)
      {
        // Check if all objects have been set.
        if (gazebo_set_state_counter_ >= static_cast<int>(objects.size()))
        {
          RCLCPP_INFO(this->get_logger(), "World is (re)set");
          rclcpp::shutdown();
          return;
        }

        // Create a request to set an object in Gazebo.
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = objects[gazebo_set_state_counter_].getname();
        request->state.pose = objects[gazebo_set_state_counter_].getPose_init();

        // Send the request asynchronously and wait for a response.
        auto result = client_ptr_->async_send_request(request);
        std::future_status status = result.wait_for(10s);  // Timeout to guarantee a graceful finish
        if (status == std::future_status::ready)
        {
          RCLCPP_INFO(this->get_logger(), "Received response");
          gazebo_set_state_counter_++;
        }
      }
      counter_++;
    }
  };

}  // namespace push_translation_gazebo

// Main function for the `push_translation_gazebo` node
int main(int argc, char * argv[])
{
  // Initialize ROS 2 communication
  rclcpp::init(argc, argv);

  // Create a shared pointer to the node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("push_translation_gazebo");

  // Create a shared pointer to a Gazebo node
  auto gazebo_node = std::make_shared<push_translation_gazebo::GazeboNode>();

  // Create a MultiThreadedExecutor to manage nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(gazebo_node);

  // Check if an argument was passed to the program
  if (argc == 2) {
    tcp_y_init = tcp_y_init - atof(argv[1]);
    RCLCPP_INFO(node->get_logger(), "init y: %f", tcp_y_init);
  }

  // Create a client to the set_desired_joint_config service
  rclcpp::Client<custom_interfaces::srv::RobConf>::SharedPtr conf_client =
    node->create_client<custom_interfaces::srv::RobConf>("/velocity_controller/set_desired_joint_config");

  // Wait for the service to be available
  while (!conf_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Solve IK and get the initial position
  std::vector<JointPos> theta_sol_init;
  myrobot.solveIK(tcp_x, tcp_y_init, tcp_z, tcp_qr, tcp_qp, tcp_qy, theta_sol_init);

  // Create a request to set the initial position
  auto init_conf_request = std::make_shared<custom_interfaces::srv::RobConf::Request>();
  if(theta_sol_init.size()!=0)
  {
    init_conf_request->event_trigger = false;
    init_conf_request->conf = {(float)theta_sol_init[0][0],(float)theta_sol_init[0][1],(float)theta_sol_init[0][2],(float)theta_sol_init[0][3],(float)theta_sol_init[0][4],(float)theta_sol_init[0][5], (float)tcp_gripper};
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No IK solution found.");
    return 0;
  }

  // Send the initial position request to the service
  auto init_conf_result = conf_client->async_send_request(init_conf_request);

  // Check if the initial position was set successfully
  if (rclcpp::spin_until_future_complete(node, init_conf_result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial pose setting was successful: %d", init_conf_result.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_desired_joint_config");
  }

  // Start spinning the executor to run nodes
  executor.spin();

  // Shutdown ROS 2 communication
  rclcpp::shutdown();
  return 0;
}
