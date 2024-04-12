#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include <math.h>

#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <custom_interfaces/srv/set_obj_id.hpp>
#include <custom_interfaces/srv/set_randomization.hpp>
#include <custom_interfaces/srv/reset_poses.hpp>
#include <custom_interfaces/msg/pose_array.hpp>
#include "custom_interfaces/srv/rob_pose.hpp"
#include "custom_interfaces/srv/rob_pose2.hpp"

//#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


using namespace std::chrono_literals;
using namespace std::placeholders;

bool gazebo_model_id_set = false;

geometry_msgs::msg::Quaternion ToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

struct EulerAngles {
    double roll, pitch, yaw;
};
EulerAngles tcp_euler;
// Code from WIkipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
EulerAngles ToEulerAngles(geometry_msgs::msg::Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    std::string frame_id;
    std::string name;
    geometry_msgs::msg::Pose pose;
    double r;
    double g;
    double b;
    double scale_x;
    double scale_y;
    double scale_z;

    bool use_embedded_materials;
public:
    inline void setObjPath(std::string p) {objPath=p;}
    inline std::string getObjPath() {return objPath;}
    inline unsigned int getTagID() {return id;}
    inline unsigned int getGazeboModelID() {return id;}
    inline std::string get_frame_id() {return frame_id;}
    inline std::string getname() {return name;}
    inline double getx() {return pose.position.x;}
    inline double gety() {return pose.position.y;}
    inline double getz() {return pose.position.z;}
    inline double getqx() {return pose.orientation.x;}
    inline double getqy() {return pose.orientation.y;}
    inline double getqz() {return pose.orientation.z;}
    inline double getqw() {return pose.orientation.w;}
    inline bool get_use_embeded_materials() {return use_embedded_materials;}
    inline double getr() {return r;}
    inline double getg() {return g;}
    inline double getb() {return b;}
    inline double getscale_x() {return scale_x;}
    inline double getscale_y() {return scale_y;}
    inline double getscale_z() {return scale_z;}

    inline geometry_msgs::msg::Pose getPose() {return pose;}

    inline void setTagID(unsigned int i) {id=i;}
    inline void setGazeboModelID(unsigned int i) {id=i;}
    inline void set_frame_id(std::string s) {frame_id=s;}
    inline void setname(std::string s) {name=s;}
    inline void setPose(geometry_msgs::msg::Pose g) {pose=g;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline void setr(double v) {r=v;}
    inline void setg(double v) {g=v;}
    inline void setb(double v) {b=v;}
    inline void setscale_x(double v) {scale_x=v;}
    inline void setscale_y(double v) {scale_y=v;}
    inline void setscale_z(double v) {scale_z=v;}

    inline void set_use_embedded_materials(bool t) {use_embedded_materials = t;}
};

std::vector<ObjectInfo> objects;

void SetWorld()
{
    ObjectInfo obj;

    objects.clear();

    obj.setObjPath("package://main_pkg/meshes/cube_tag0_grey.dae");
    obj.setTagID(0);
    obj.setGazeboModelID(0);
    obj.setname("cube_tag0_grey");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.2);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag1_violet.dae");
    obj.setTagID(1);
    obj.setGazeboModelID(1);
    obj.setname("cube_tag1_violet");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.2);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag2_indigo.dae");
    obj.setTagID(2);
    obj.setGazeboModelID(2);
    obj.setname("cube_tag2_indigo");
    obj.set_frame_id("base_link");
    obj.setx(0.3);
    obj.sety(0.2);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag3_blue.dae");
    obj.setTagID(3);
    obj.setGazeboModelID(3);
    obj.setname("cube_tag3_blue");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.1);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag4_green.dae");
    obj.setTagID(4);
    obj.setGazeboModelID(4);
    obj.setname("cube_tag4_green");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.1);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag5_yellow.dae");
    obj.setTagID(5);
    obj.setGazeboModelID(5);
    obj.setname("cube_tag5_yellow");
    obj.set_frame_id("base_link");
    obj.setx(0.3);
    obj.sety(0.1);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag6_orange.dae");
    obj.setTagID(6);
    obj.setGazeboModelID(6);
    obj.setname("cube_tag6_orange");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.0);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag7_red.dae");
    obj.setTagID(7);
    obj.setGazeboModelID(7);
    obj.setname("cube_tag7_red");
    obj.set_frame_id("base_link");
    obj.setx(-0.1);
    obj.sety(0.8);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.0);
    obj.setg(0.0);
    obj.setb(0.9);
    objects.push_back(obj);
}

tf2::Quaternion quaternion_multiply(tf2::Quaternion q0,tf2::Quaternion q1){


   // Extract the values from q0
   double w0 = q0.w();
   double x0 = q0.x();
   double y0 = q0.y();
   double z0 = q0.z();

   // Extract the values from q1
   double w1 = q1.w();
   double x1 = q1.x();
   double y1 = q1.y();
   double z1 = q1.z();

   // Computer the product of the two quaternions, term by term
   tf2::Quaternion final_quaternion;
   final_quaternion[0] = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
   final_quaternion[1] = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
   final_quaternion[2] = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
   final_quaternion[3] = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;

   //normalize
   final_quaternion.normalize();

   RCLCPP_INFO(rclcpp::get_logger("rclccp"), "multiplied quaternion: qx=%f, qy=%f, qz=%f, qw=%f", final_quaternion.x(),final_quaternion.y(),final_quaternion.z(),final_quaternion.w());

   // Return a normalized quaternion
   return final_quaternion;

}

class WorldSetup : public rclcpp::Node
{
public:
  WorldSetup() : Node("world_setup"), push_cube_id_(5)
  {
    srand ( time(NULL) );
    SetWorld();
    RCLCPP_INFO(this->get_logger(), "World is setup! Loaded %d object(s)", (int)objects.size());

    // Set initial values for variables
    topic_ = "visualization_marker_array";
    rate_ = 1;

    // Create publisher and subscriber
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_, rate_);
    state_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo_state/model_states", rate_, std::bind(&WorldSetup::update_marker_array, this, _1));

    client_gazebo_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_controller_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //attach_service_ = this->create_service<custom_interfaces::srv::SetObjId>("/world_setup/attach_obj", std::bind(&WorldSetup::attach_obj, this, _1, _2));
    //reset_service_ = this->create_service<custom_interfaces::srv::SetRandomization>("/world_setup/reset_world", std::bind(&WorldSetup::reset_world, this, _1, _2));
    reset_service_ = this->create_service<custom_interfaces::srv::ResetPoses>("/world_setup/reset_world", std::bind(&WorldSetup::reset_world, this, _1, _2));
    reset_cubes_service_ = this->create_service<custom_interfaces::srv::ResetPoses>("/world_setup/reset_cubes", std::bind(&WorldSetup::reset_cubes, this, _1, _2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Create callback groups for timer and client.


    // Create a timer to send requests to Gazebo.
    //timer_ptr_ = this->create_wall_timer(10ms, std::bind(&WorldSetup::timer_callback, this), timer_cb_group_);

    // Create a client to send requests to Gazebo.
    client_gazebo_ptr_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
      "/gazebo_state/set_entity_state",
      rmw_qos_profile_services_default,
      client_gazebo_cb_group_);

    // Create a client to send requests to Gazebo.
    client_controller_ptr_ = this->create_client<custom_interfaces::srv::RobPose2>(
      "/velocity_controller/set_desired_rob_pose",
      rmw_qos_profile_services_default,
      client_controller_cb_group_);

    //double x,y,yaw;
    initial_cube_pose_.position.x = -0.1;
    initial_cube_pose_.position.y = 0.6;
    initial_cube_pose_.orientation = ToQuaternion(0,M_PI,0);
    initial_robot_pose_ = {{0.1,-0.5,0.132,0,M_PI,0,0.6},               //cube y goal > cube y >0.6
                            {0.2,-0.6,0.132,0,M_PI,-M_PI_2,0.6},        //cube x goal > cube x > -0.1
                            {0.1,-0.7,0.132,0,M_PI,M_PI,0.6},           //cube y goal < cube y < 0.6
                            {-0.05,-0.6,0.132,0,M_PI,M_PI_2,0.6}};        //cube x goal < cube x < -0.1
  }


private:
  // Declare member variables
  rclcpp::CallbackGroup::SharedPtr client_gazebo_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_controller_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_ptr_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_gazebo_ptr_;
  rclcpp::Client<custom_interfaces::srv::RobPose2>::SharedPtr client_controller_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr state_subscriber_;
  rclcpp::Service<custom_interfaces::srv::ResetPoses>::SharedPtr reset_service_;
  rclcpp::Service<custom_interfaces::srv::ResetPoses>::SharedPtr reset_cubes_service_;
  std::string topic_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;//{nullptr}
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  int rate_;
  geometry_msgs::msg::Pose initial_cube_pose_;
  std::vector<std::vector<double>> initial_robot_pose_;
  geometry_msgs::msg::TransformStamped t_cube_eef_;
  tf2::Quaternion quat_;
  int push_cube_id_;
  bool attached_;

  //! Reset the world
  /*void reset_world(std::shared_ptr<custom_interfaces::srv::SetRandomization::Request>  req,
           std::shared_ptr<custom_interfaces::srv::SetRandomization::Response> res)
  {

    int n_rand = 1000;
    double range_cube = 0.5; //m
    double range_robot = 0.3; //m
    double range_yaw = M_PI_2;
    std::vector<int> acts = {1,3};
    int sit = acts[rand() % 2];
    //RCLCPP_INFO(this->get_logger(), "sit: %d", sit);
    double rand_f = req->rand_factor;
    double cube_yaw = (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_yaw*rand_f;
    double cube_x = initial_cube_pose_.position.x + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_cube*rand_f;
    double cube_y = initial_cube_pose_.position.y + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_cube*rand_f;
    double eef_yaw = initial_robot_pose_[sit][5] + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_yaw*rand_f;
    double eef_x = initial_robot_pose_[sit][0] + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_robot*rand_f;
    double eef_y = initial_robot_pose_[sit][1] + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_robot*rand_f;

    double cube_goal_yaw = cube_yaw + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_yaw*rand_f;
    double cube_goal_x = cube_x + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_cube*rand_f;
    double cube_goal_y = cube_y + (float)(n_rand/2-rand() % n_rand)/n_rand/2*range_cube*rand_f;


    // Wait until the Gazebo service is available.
    while (!client_controller_ptr_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the gazebo set object pose service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto request_controller = std::make_shared<custom_interfaces::srv::RobPose::Request>();
    request_controller->cart_pose = 1;
    request_controller->goal_dir = initial_robot_pose_[sit];
    request_controller->goal_dir[0] = eef_x;
    request_controller->goal_dir[1] = eef_y;
    request_controller->goal_dir[5] = eef_yaw;
    request_controller->timeout_ms = 1000; //allow one second to reach the goal pose
    // Send the request asynchronously and wait for a response.
    auto result_controller = client_controller_ptr_->async_send_request(request_controller);
    std::future_status status_controller = result_controller.wait_for(10s);  // Timeout to guarantee a graceful finish
    if (status_controller == std::future_status::ready)
    {
      //RCLCPP_INFO(this->get_logger(), "Received response");
      RCLCPP_INFO(this->get_logger(), "robot pose reset");
    }

    // Wait until the Gazebo service is available.
    while (!client_gazebo_ptr_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the gazebo set object pose service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    rclcpp::sleep_for(1s); //wait for robot placement before setting the cube

    auto request_gazebo = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request_gazebo->state.name = objects[push_cube_id_].getname();
    request_gazebo->state.pose = initial_cube_pose_;
    request_gazebo->state.pose.position.x = cube_x;
    request_gazebo->state.pose.position.y = cube_y;

    request_gazebo->state.pose.orientation = ToQuaternion(0,M_PI,cube_yaw);
    // Send the request asynchronously and wait for a response.
    auto result_gazebo = client_gazebo_ptr_->async_send_request(request_gazebo);
    std::future_status status_gazebo = result_gazebo.wait_for(10s);  // Timeout to guarantee a graceful finish
    if (status_gazebo == std::future_status::ready)
    {
      //RCLCPP_INFO(this->get_logger(), "Received response");
      RCLCPP_INFO(this->get_logger(), "cube pose reset: %d",push_cube_id_);
    }

    res->success = true;
    res->cube_current = {cube_x*-100,cube_y*-100,cube_yaw}; //convert x,y to cm to better match rad dimensions
    res->cube_goal = {cube_goal_x*-100,cube_goal_y*-100,cube_goal_yaw}; //convert x,y to cm to better match rad dimensions
    res->eef_current = {eef_x*100,eef_y*100,eef_yaw}; //convert x and y to coordinate frame of the cube and to cm to better match the rad dimensions
    RCLCPP_INFO(this->get_logger(), "world is reset: cube init x = %f, cube goal x = %f",cube_x*-100,cube_goal_x*-100);

  }*/

  void reset_world(std::shared_ptr<custom_interfaces::srv::ResetPoses::Request>  req,
           std::shared_ptr<custom_interfaces::srv::ResetPoses::Response> res)
  {
    double pose_gripper = req->pose_gripper;
    geometry_msgs::msg::Pose pose_eef;
    geometry_msgs::msg::Pose pose_cube;
    pose_eef = req->pose_eef;
    EulerAngles euler_eef = ToEulerAngles(pose_eef.orientation);
    pose_cube = req->pose_cube;
    int initial_reset = req->initial_reset;
    push_cube_id_ = req->cube_id;

    // Wait until the Gazebo service is available.
    while (!client_controller_ptr_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the gazebo set object pose service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gazebo Service not available, waiting again...");
    }

    auto request_controller = std::make_shared<custom_interfaces::srv::RobPose2::Request>();
    request_controller->cart_pose = 1;
    request_controller->goal_dir = {pose_eef.position.x,pose_eef.position.y,pose_eef.position.z,euler_eef.roll,euler_eef.pitch,euler_eef.yaw,pose_gripper};
    //equest_controller->goal_dir[0] = pose_eef.position.x;
    //request_controller->goal_dir[1] = pose_eef.position.y;
    //request_controller->goal_dir[5] = euler_eef.yaw;
    request_controller->timeout_ms = 1000; //allow one second to reach the goal pose
    // Send the request asynchronously and wait for a response.
    auto result_controller = client_controller_ptr_->async_send_request(request_controller);
    std::future_status status_controller = result_controller.wait_for(10s);  // Timeout to guarantee a graceful finish
    if (status_controller == std::future_status::ready)
    {
      //RCLCPP_INFO(this->get_logger(), "Received response");
      RCLCPP_INFO(this->get_logger(), "robot pose reset");
    }

    // Wait until the Gazebo service is available.
    while (!client_gazebo_ptr_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the gazebo set object pose service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gazebo Service not available, waiting again...");
    }

    if(initial_reset){
      rclcpp::sleep_for(2s); //wait for robot placement before setting the cube
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial reset for 2s triggered");
    }
    else{
      rclcpp::sleep_for(1s); //wait for robot placement before setting the cube
    }


    auto request_gazebo = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request_gazebo->state.name = objects[push_cube_id_].getname();
    request_gazebo->state.pose = pose_cube;

    //request_gazebo->state.pose.orientation = ToQuaternion(0,M_PI,cube_yaw);
    // Send the request asynchronously and wait for a response.
    auto result_gazebo = client_gazebo_ptr_->async_send_request(request_gazebo);
    std::future_status status_gazebo = result_gazebo.wait_for(10s);  // Timeout to guarantee a graceful finish
    if (status_gazebo == std::future_status::ready)
    {
      //RCLCPP_INFO(this->get_logger(), "Received response");
      RCLCPP_INFO(this->get_logger(), "cube pose reset: %d",push_cube_id_);
    }

    res->success = true;
    //res->cube_current = {cube_x*-100,cube_y*-100,cube_yaw}; //convert x,y to cm to better match rad dimensions
    //res->cube_goal = {cube_goal_x*-100,cube_goal_y*-100,cube_goal_yaw}; //convert x,y to cm to better match rad dimensions
    //res->eef_current = {eef_x*100,eef_y*100,eef_yaw}; //convert x and y to coordinate frame of the cube and to cm to better match the rad dimensions
    RCLCPP_INFO(this->get_logger(), "world is reset");

  }

  void reset_cubes(std::shared_ptr<custom_interfaces::srv::ResetPoses::Request>  req,
           std::shared_ptr<custom_interfaces::srv::ResetPoses::Response> res)
  {
    geometry_msgs::msg::Pose pose_cube;
    pose_cube = req->pose_cube;

    // Wait until the Gazebo service is available.
    while (!client_gazebo_ptr_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the gazebo set object pose service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto request_gazebo = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request_gazebo->state.name = objects[push_cube_id_].getname();
    request_gazebo->state.pose = pose_cube;

    //request_gazebo->state.pose.orientation = ToQuaternion(0,M_PI,cube_yaw);
    // Send the request asynchronously and wait for a response.
    auto result_gazebo = client_gazebo_ptr_->async_send_request(request_gazebo);
    std::future_status status_gazebo = result_gazebo.wait_for(10s);  // Timeout to guarantee a graceful finish
    if (status_gazebo == std::future_status::ready)
    {
      //RCLCPP_INFO(this->get_logger(), "Received response");
      RCLCPP_INFO(this->get_logger(), "cube pose reset: %d",push_cube_id_);
    }

    res->success = true;
    //res->cube_current = {cube_x*-100,cube_y*-100,cube_yaw}; //convert x,y to cm to better match rad dimensions
    //res->cube_goal = {cube_goal_x*-100,cube_goal_y*-100,cube_goal_yaw}; //convert x,y to cm to better match rad dimensions
    //res->eef_current = {eef_x*100,eef_y*100,eef_yaw}; //convert x and y to coordinate frame of the cube and to cm to better match the rad dimensions
    RCLCPP_INFO(this->get_logger(), "cubes are reset");

  }

  // Callback function for state subscriber
  void update_marker_array(gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    // If gazebo model IDs have not been set yet, set them
    if (!gazebo_model_id_set)
    {
      std::vector<std::string> names = msg->name;


      for (int i = 0; i < (int)names.size(); i++)
      {
        for (int j = 0; j < (int)objects.size(); j++)
        {
          if (objects[j].getname().compare(names[i]) == 0)
          {
            objects[j].setGazeboModelID(i);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "name: %s, id: %d!", names[i].c_str(), i);
          }
        }
      }

      gazebo_model_id_set = true;
    }

    // Update markers for each object
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markers;

    for (int i = 0; i < (int)objects.size(); i++) {
      // Set the pose of the current object
      objects[i].setPose(msg->pose[objects[i].getGazeboModelID()]);

      // Create a new marker for the current object
      marker.header.frame_id = objects[i].get_frame_id();
      marker.header.stamp = this->get_clock()->now();
      marker.ns = objects[i].getname();//"cubes";
      marker.id = objects[i].getTagID();
      marker.mesh_resource = objects[i].getObjPath();
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = objects[i].getPose();
      marker.scale.x = objects[i].getscale_x();
      marker.scale.y = objects[i].getscale_y();
      marker.scale.z = objects[i].getscale_z();
      marker.color.a = 1.0;
      marker.color.r = objects[i].getr();
      marker.color.g = objects[i].getg();
      marker.color.b = objects[i].getb();
      marker.mesh_use_embedded_materials = objects[i].get_use_embeded_materials();

      // Add the marker to the list of markers
      markers.markers.push_back(marker);

      geometry_msgs::msg::TransformStamped t;

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = objects[i].get_frame_id();
      t.child_frame_id = objects[i].getname();

      t.transform.translation.x = objects[i].getx();
      t.transform.translation.y = objects[i].gety();
      t.transform.translation.z = objects[i].getz();

      //tf2::Quaternion q;
      //q.setRPY(0, 0, msg->theta);
      t.transform.rotation.x = objects[i].getqx();
      t.transform.rotation.y = objects[i].getqy();
      t.transform.rotation.z = objects[i].getqz();
      t.transform.rotation.w = objects[i].getqw();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    // Publish the markers
    publisher_->publish(markers);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorldSetup>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  //rclcpp::spin(std::make_shared<WorldSetup>());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
