// align cubes in a row -> set model states (client call)
// move robot into start position (in front of cube0 client call)
// push the cube0 into a gap (client call?)
// move back to the next cube1 and push it into a gap
// compare the completion time
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include <math.h>
#include <string.h>
#include <time.h>

#include "custom_interfaces/srv/rob_conf.hpp"
#include "custom_interfaces/srv/delayed_pose.hpp"


#include <gazebo_msgs/msg/model_states.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <kinenik/kinenik_ur.h>

#include <iostream>
#include <fstream>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <gazebo_msgs/srv/get_entity_state.hpp>

#include <dmp/Trajectory.hpp>

using namespace std::chrono_literals;

//###################   General parameters  ############################
int n_runs = 4;
std::vector<double> goal_states;
double average = 0;
int latency_ms = 1000;
//std::chrono::milliseconds latency_ms = 0ms;
std::ofstream myfile_gazebo;
std::string filename_gazebo;
std::ofstream myfile_control;
std::string filename_control;
std::ofstream myfile;
std::string filename;

std::vector<std::vector<double>> history;
std::vector<std::vector<double>> history_gazebo;
std::vector<std::vector<double>> history_control;

double cube_y_init = 0.65;

int initial_cube_id = 3;

double x_offset = -0.027;

double tcp_x = 0.1 +x_offset;// + x_rot_offset;
//double tcp_y_init = -0.5;
//double tcp_y_init2 = -0.59;//-0.61;
//double tcp_y = tcp_y_init;
//double tcp_z = 0.132;
//double tcp_qr = 0;
double tcp_qp = M_PI;
double tcp_qy = 0;
double tcp_gripper = 0.6;
double tcp_y_init = -0.39;
double tcp_y_init2 = -0.48;//-0.61;
double tcp_y = tcp_y_init;
double tcp_z = 0.08;
double tcp_qr = 0.8;

double cube_y_goal = 0.7;

std::string reference_frame = "base_link";

int rate = 100;
double step_size = 0.001;
rclcpp::Clock clk;
rclcpp::Time time_pose_sensed;
rclcpp::Time time_pose_delayed;
rclcpp::Time time_eef;
double seconds_pose_sensed;
double seconds_pose_delayed;
double seconds_eef;
int secs;
rclcpp::Time time_pose_at_finished_pushing;

KinenikUR myrobot("UR5e");
geometry_msgs::msg::Pose get_model_state_pose;
geometry_msgs::msg::Pose get_model_state_pose_delayed;
int get_eef_y;

bool gazebo_model_id_set = false;
int back_up_counter = 0;
int back_up_counter_limit = 30;

geometry_msgs::msg::TransformStamped transform_ur_eef;
std::vector<double> tcp_pose;
std::vector<double> ik_offsets = {0.025,-0.002,0.055};
//std::vector<double> ik_offsets = {0.0,-0.0,0.0};
std::vector<double> multipliers = {-1,-1,1};

std::vector<double> latency_distribution;
std::vector<int> latency_distribution_micro;
int latency_distribution_size;
std::string network_directory = "docs/data/AAU_latency/";
std::string network = "private5g";
bool trigger = 0;

struct EulerAngles {
    double roll, pitch, yaw;
};
EulerAngles tcp_euler;
EulerAngles get_model_state_euler;
EulerAngles get_model_state_euler_delayed;
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
    obj.setx(-0.4);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());

    objects.push_back(obj);

    obj.setTagID(1);
    obj.setGazeboModelID(1);
    obj.setname("cube_tag1_violet");
    obj.setx(-0.3);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(2);
    obj.setGazeboModelID(2);
    obj.setname("cube_tag2_indigo");
    obj.setx(-0.2);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(3);
    obj.setGazeboModelID(3);
    obj.setname("cube_tag3_blue");
    obj.setx(-0.1);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(4);
    obj.setGazeboModelID(4);
    obj.setname("cube_tag4_green");
    obj.setx(0.0);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(5);
    obj.setGazeboModelID(5);
    obj.setname("cube_tag5_yellow");
    obj.setx(0.1);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(6);
    obj.setGazeboModelID(6);
    obj.setname("cube_tag6_orange");
    obj.setx(0.2);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);

    obj.setTagID(7);
    obj.setGazeboModelID(7);
    obj.setname("cube_tag7_red");
    obj.setx(0.3);
    obj.sety(cube_y_init);
    obj.setz(0.025);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setPose_init(obj.getPose());
    objects.push_back(obj);
}


namespace push_reaction_main
{
class GazeboUpdateNode : public rclcpp::Node
{
  public:
    GazeboUpdateNode() : Node("gazebo_update_node")//, state_service_ready_(false), state_received_(false)
    {

      srand ( time(NULL) );
      SetWorld();

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // timer receives pose 30 times/second (similar to camera)
      timer_ptr_ = this->create_wall_timer(17ms, std::bind(&GazeboUpdateNode::timer_callback, this),timer_cb_group_);
      client_ptr_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo_state/get_entity_state",rmw_qos_profile_services_default,client_cb_group_);

      //RCLCPP_INFO(this->get_logger(), "Sending request");
      while (!client_ptr_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

    }


  private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_ptr_;

    //int counter_;

    void timer_callback()
    {
      auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
      //auto delay_request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
      request->name = objects[3].getname();
      //RCLCPP_INFO(this->get_logger(), "Sending request");
      auto result = client_ptr_->async_send_request(request);
      std::future_status status = result.wait_for(10s);  // timeout to guarantee a graceful finish
      if (status == std::future_status::ready) {
        //RCLCPP_INFO(this->get_logger(), "Received response");
        get_model_state_pose = result.get()->state.pose;
        get_model_state_euler = ToEulerAngles(get_model_state_pose.orientation);

        time_pose_sensed = clk.now();
        seconds_pose_sensed = time_pose_sensed.seconds()-secs;
        //RCLCPP_INFO(this->get_logger(), "stamp set: %f", time_pose_sensed.seconds());
        latency_ms = latency_distribution_micro[rand() % latency_distribution_size];
        rclcpp::sleep_for(std::chrono::microseconds(latency_ms));
        time_pose_delayed = clk.now();
        seconds_pose_delayed = time_pose_delayed.seconds()-secs;
        //RCLCPP_INFO(this->get_logger(), "delayed stamp set: %f", time_pose_delayed.seconds());
        get_model_state_pose_delayed = get_model_state_pose;
        get_model_state_euler_delayed = ToEulerAngles(get_model_state_pose_delayed.orientation);
        //RCLCPP_INFO(this->get_logger(), "delayed pose set");
        //RCLCPP_INFO(this->get_logger(), "pose set, x: %f, y: %f", get_model_state_pose.position.x, get_model_state_pose.position.y);
        //history_gazebo.push_back({seconds_pose_sensed,seconds_pose_delayed,(float)latency_ms,get_model_state_pose.position.x,get_model_state_pose.position.y,get_model_state_euler.yaw});
      }
    }
};

class RobotPushNode : public rclcpp::Node
{
  public:
    RobotPushNode() : Node("robot_push_node"), tcp_y_(tcp_y_init)//, state_service_ready_(false), state_received_(false)
    {
      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      timer_ptr_ = this->create_wall_timer(5ms, std::bind(&RobotPushNode::timer_callback, this),timer_cb_group_);
      client_ptr_ = this->create_client<custom_interfaces::srv::RobConf>("/velocity_controller/set_desired_joint_config",rmw_qos_profile_services_default,client_cb_group_);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initial robot config: (%f,%f,%f,%f,%f,%f)", tcp_x, tcp_y_, tcp_z, tcp_qr, tcp_qp, tcp_qr);
      myrobot.solveIK(tcp_x, tcp_y_init, tcp_z, tcp_qr, tcp_qp, tcp_qy, theta_sol_);

      while (!client_ptr_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

    }


  private:

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Client<custom_interfaces::srv::RobConf>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double tcp_y_;
    double tcp_x_;
    std::vector<JointPos> theta_sol_;
    std::vector<JointPos> theta_sol_previous_;

    void timer_callback()
    {
      try {
        transform_ur_eef = tf_buffer_->lookupTransform(
          reference_frame, "wrist_3_link",
          tf2::TimePointZero);
        //time_eef = clk.now();
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          reference_frame.c_str(), "wrist_3_link", ex.what());
        return;
      }


      theta_sol_previous_ = theta_sol_;

      tcp_euler = ToEulerAngles(transform_ur_eef.transform.rotation);
      tcp_pose = {(transform_ur_eef.transform.translation.x*multipliers[0])-ik_offsets[0],(transform_ur_eef.transform.translation.y*multipliers[1])-ik_offsets[1],(transform_ur_eef.transform.translation.z*multipliers[2])-ik_offsets[2],tcp_euler.roll,tcp_euler.pitch,tcp_euler.yaw,tcp_gripper};

      time_eef = clk.now();
      seconds_eef = time_eef.seconds()-secs;

      if(get_model_state_pose_delayed.position.y >= cube_y_goal){
        trigger = 1;
        //history_control.push_back({seconds_eef,tcp_pose[0],tcp_pose[1],tcp_pose[2],tcp_pose[5],get_model_state_pose_delayed.position.y,tcp_y_,trigger});
        tcp_y_ += step_size;
        myrobot.solveIK(tcp_x, tcp_y_, tcp_z, tcp_qr, tcp_qp, tcp_qy, theta_sol_);
        auto final_request = std::make_shared<custom_interfaces::srv::RobConf::Request>();
        final_request->event_trigger = true;
        final_request->time_stamp = time_pose_delayed.seconds();
        //final_request->conf = {(float)theta_sol_previous_[0][0],(float)theta_sol_previous_[0][1],(float)theta_sol_previous_[0][2],(float)theta_sol_previous_[0][3],(float)theta_sol_previous_[0][4],(float)theta_sol_previous_[0][5], (float)tcp_gripper};
        final_request->conf = {(float)theta_sol_[0][0],(float)theta_sol_[0][1],(float)theta_sol_[0][2],(float)theta_sol_[0][3],(float)theta_sol_[0][4],(float)theta_sol_[0][5], (float)tcp_gripper};
        auto final_result = client_ptr_->async_send_request(final_request);
        std::future_status final_status = final_result.wait_for(10s);  // timeout to guarantee a graceful finish
        if (final_status == std::future_status::ready) {
          RCLCPP_INFO(this->get_logger(), "Object reached goal %f, back up!, %d", cube_y_goal, back_up_counter);
          back_up_counter++;
          //rclcpp::sleep_for(100ms);
        }


      }
      else{
        trigger = 0;
        //history_control.push_back({seconds_eef,tcp_pose[0],tcp_pose[1],tcp_pose[2],tcp_pose[5],get_model_state_pose_delayed.position.y,tcp_y_,trigger,seconds_pose_sensed,seconds_pose_delayed,(float)latency_ms,get_model_state_pose.position.x,get_model_state_pose.position.y,get_model_state_euler.yaw});
        tcp_y_ -= step_size;
        myrobot.solveIK(tcp_x, tcp_y_, tcp_z, tcp_qr, tcp_qp, tcp_qy, theta_sol_);
        //RCLCPP_INFO(this->get_logger(), "tcp_y: %f, cube y:%f, cube delayed y: %f", tcp_y_, get_model_state_pose.position.y, get_model_state_pose_delayed.position.y);
        auto request = std::make_shared<custom_interfaces::srv::RobConf::Request>();
        //auto delay_request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        if(theta_sol_.size()!=0)
        {
          request->event_trigger = false;
          request->time_stamp = time_pose_delayed.seconds();
          request->conf = {(float)theta_sol_[0][0],(float)theta_sol_[0][1],(float)theta_sol_[0][2],(float)theta_sol_[0][3],(float)theta_sol_[0][4],(float)theta_sol_[0][5], (float)tcp_gripper};
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "No IK solution found.");
          rclcpp::shutdown();
          return;
        }
        auto result = client_ptr_->async_send_request(request);
        std::future_status status = result.wait_for(10s);  // timeout to guarantee a graceful finish
        if (status == std::future_status::ready) {
            //RCLCPP_INFO(this->get_logger(), "Received response");
        }
      }

      history.push_back({seconds_eef,tcp_pose[0],tcp_pose[1],tcp_pose[2],tcp_pose[5],tcp_y_,(float)trigger,seconds_pose_sensed,seconds_pose_delayed,(float)latency_ms,get_model_state_pose.position.x,get_model_state_pose.position.y,get_model_state_euler.yaw});
      if(back_up_counter > back_up_counter_limit){
        rclcpp::shutdown();
        return;
      }
    }

};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("push_react_main");

  // Set the rate parameter from the command line
  if (argc == 5) {
    tcp_y_init = tcp_y_init - atof(argv[1]);
    cube_y_goal= cube_y_init + atof(argv[2]);
    //latency_ms = atoi(argv[3]);
    network = argv[3];
    step_size = atof(argv[4]);
    RCLCPP_INFO(node->get_logger(), "netork: %s, target y: %f, tcp_y_init: %f, step_size: %f ", network.c_str(), cube_y_goal, tcp_y_init, step_size);
  }

  auto gazebo_update_node = std::make_shared<push_reaction_main::GazeboUpdateNode>();
  auto push_node = std::make_shared<push_reaction_main::RobotPushNode>();
  //auto robot_setup_node = std::make_shared<push_experiment_02::RobotSetupNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(gazebo_update_node);
  executor.add_node(push_node);
  rclcpp::Clock clk;
  rclcpp::Time start_time;
  start_time = clk.now();
  secs = start_time.seconds();

  //read latency data
  std::string network_filename = network_directory + network.c_str() + ".csv";
  RCLCPP_INFO(node->get_logger(), "Network data file name: %s", network_filename.c_str());
  std::ifstream  data(network_filename);
  if(data.is_open())
  {
    std::string line;
    while(std::getline(data, line)){
      latency_distribution.emplace_back(std::stold(line));
    }
  }
  latency_distribution_size = (int)latency_distribution.size();
  RCLCPP_INFO(node->get_logger(), "Convert to microseconds, size: %d ", latency_distribution_size);

  for(int i = 0; i<latency_distribution_size; i++){
    latency_distribution_micro.push_back(latency_distribution[i]*1000);
  }
  RCLCPP_INFO(node->get_logger(), "Finsihed reading latency data for netork %s: %d elements, %f, %d", network.c_str(), latency_distribution_size,latency_distribution[0],latency_distribution_micro[0]);

  filename_gazebo = "docs/data/push_translation/gazebo_node/" + std::to_string(secs) + "_" + network.c_str() + "_"  + std::to_string(step_size) + "steps" + ".csv";
  filename_control = "docs/data/push_translation/control_node/" + std::to_string(secs) + "_" + network.c_str() + "_" + std::to_string(step_size) + "steps" + ".csv";
  filename = "docs/data/push_translation/" + std::to_string(secs) + "_" + network.c_str() + "_"  + std::to_string(step_size) + "steps" + ".csv";
  //myfile_gazebo.open(filename_gazebo);
  //myfile_control.open(filename_control);
  myfile.open(filename);
  myfile << "secs_eef,tcp_x,tcp_y,tcp_z,tcp_yaw,tcp_y_,trigger,secs_sensed,secs_delayed,latency,cube_x,cube_y,cube_yaw\n";
  executor.spin();

  /*for(int i=0;i<(int)history_gazebo.size();i++){
    for(int j=0;j<(int)history_gazebo[0].size();j++){
      myfile_gazebo << history_gazebo[i][j] << ",";
    }
    myfile_gazebo << "\n";
  }

  for(int i=0;i<(int)history_control.size();i++){
    for(int j=0;j<(int)history_control[0].size();j++){
      myfile_control << history_control[i][j] << ",";
    }
    myfile_control << "\n";
  }*/

  for(int i=0;i<(int)history.size();i++){
    for(int j=0;j<(int)history[0].size();j++){
      myfile << history[i][j] << ",";
    }
    myfile << "\n";
  }

  //myfile_gazebo.close();
  //myfile_control.close();
  myfile.close();

  rclcpp::shutdown();
  return 0;
}
