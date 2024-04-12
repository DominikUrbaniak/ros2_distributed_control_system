#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include <math.h>
#include <string.h>
#include <time.h>

#include <iostream>
#include <fstream>

#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <custom_interfaces/msg/pose_sensing.hpp>
#include <custom_interfaces/srv/pose_sensing_settings.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

int framerate = 60;
int frame_rate_microsec = (float)1/framerate*1000*1000;
std::string dir_path = "docs/data/edge/";
std::string network_directory = "src/latency_distributions/";

std::string network = "private5g";
int set_delay_mode = 0; //1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
int computation_latency = 170000;
int cube_id = 5;
int n_freshness_samples = 10;
std::string qos_profile_key = "R10";
int lost_packets = 0;
std::vector<double> latency_distribution;

std::vector<double> latency_vector_fresh;
std::vector<int> latency_distribution_micro;
int latency_distribution_size;

rclcpp::Clock clk;
rclcpp::Time time_pose_sensed;
rclcpp::Time time_pose_delayed;
long nano_seconds_pose_sensed;
long nano_seconds_pose_delayed;

geometry_msgs::msg::Pose get_model_state_pose;
geometry_msgs::msg::Pose get_model_state_pose_delayed;

int latency_sample;
int latency_mms;
int previous_latency;
int best_latency;

// Define QoS profiles
rclcpp::QoS qos_profile_b10 = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
rclcpp::QoS qos_profile_r10 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
rclcpp::QoS qos_profile_r1 = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
rclcpp::QoS qos_profile_b1 = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
rclcpp::QoS selected_qos_profile = qos_profile_r10;

class GazeboUpdateNode : public rclcpp::Node
{
  public:
    GazeboUpdateNode() : Node("gazebo_update_node")//, state_service_ready_(false), state_received_(false)
    {
      srand ( time(NULL) );
      objects_ = {"cube_tag0_grey","cube_tag1_violet","cube_tag2_indigo","cube_tag3_blue","cube_tag4_green","cube_tag5_yellow","cube_tag6_orange","cube_tag7_red"};
      //networks_ = {"ethernet","private5g_urllc","private5g","private4g","wifi5_ideal","wifi5_loaded"};
      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // timer receives pose 30 times/second (similar to camera)
      //timer_ptr_ = this->create_wall_timer(std::chrono::microseconds(frame_rate_microsec), std::bind(&GazeboUpdateNode::timer_callback, this),timer_cb_group_);
      client_ptr_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo_state/get_entity_state",rmw_qos_profile_services_default,client_cb_group_);

      //pose_service_ = this->create_service<custom_interfaces::srv::PoseSensing>("/pose_sensing/get_delayed_pose", std::bind(&GazeboUpdateNode::get_delayed_pose, this, _1, _2));
      //pose_publisher_ = this->create_publisher<custom_interfaces::msg::PoseSensing>("/pose_sensing/delayed_pose", 10); //10 is the queue size
      settings_service_ = this->create_service<custom_interfaces::srv::PoseSensingSettings>("/pose_sensing/set_settings", std::bind(&GazeboUpdateNode::set_pose_sesing_settings, this, _1, _2));

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
    //rclcpp::Service<custom_interfaces::srv::PoseSensing>::SharedPtr pose_service_;
    rclcpp::Publisher<custom_interfaces::msg::PoseSensing>::SharedPtr pose_publisher_;
    rclcpp::Service<custom_interfaces::srv::PoseSensingSettings>::SharedPtr settings_service_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_ptr_;
    std::vector<std::string> objects_;
    custom_interfaces::msg::PoseSensing message_;
    //std::vector<std::string> networks_;

    //int counter_;

    void timer_callback()
    {
      auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
      //auto delay_request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
      request->name = objects_[cube_id];
      //RCLCPP_INFO(this->get_logger(), "Sending request");
      auto result = client_ptr_->async_send_request(request);
      std::vector<double> latency_vector;
      std::future_status status = result.wait_for(10s);  // timeout to guarantee a graceful finish
      if (status == std::future_status::ready) {
        //RCLCPP_INFO(this->get_logger(), "Received response");
        get_model_state_pose = result.get()->state.pose;
        //get_model_state_euler = ToEulerAngles(get_model_state_pose.orientation);

        time_pose_sensed = clk.now();

        //RCLCPP_INFO(this->get_logger(), "stamp set: %f", time_pose_sensed.seconds());
        if(set_delay_mode==1){
          for(int i=0;i<n_freshness_samples;i++){
            latency_sample = latency_distribution[rand() % latency_distribution_size]*1000;
            latency_vector.push_back(latency_sample);
          }
          //RCLCPP_INFO(this->get_logger(), "last latency sample: %d", latency_sample);
          previous_latency = latency_vector[0];
          best_latency = previous_latency;
          lost_packets = 0;
          for(int i=1;i<n_freshness_samples;i++){
            if(previous_latency>latency_vector[i]+frame_rate_microsec*(i)){
              if(best_latency>latency_vector[i]+frame_rate_microsec*(i)){
                best_latency = latency_vector[i]+frame_rate_microsec*(i);
                lost_packets = i;
              }
            }
          }
          latency_mms = best_latency;
          //RCLCPP_INFO(this->get_logger(), "best latency: %d, latency_distribution[0]: %f", latency_mms,latency_distribution[0]);
          rclcpp::sleep_for(std::chrono::microseconds(latency_mms));
        }
        else if(set_delay_mode==2){
          for(int i=0;i<n_freshness_samples;i++){
            latency_sample = latency_distribution[rand() % latency_distribution_size]*1000;
            latency_vector.push_back(latency_sample);
          }
          previous_latency = latency_vector[0];
          best_latency = previous_latency;
          lost_packets = 0;
          for(int i=1;i<n_freshness_samples;i++){
            if(previous_latency>latency_vector[i]+frame_rate_microsec*(i)){
              if(best_latency>latency_vector[i]+frame_rate_microsec*(i)){
                best_latency = latency_vector[i]+frame_rate_microsec*(i);
                lost_packets = i;
              }
            }
          }
          latency_mms = computation_latency + best_latency;
          rclcpp::sleep_for(std::chrono::microseconds(latency_mms));
        }
        else if(set_delay_mode==3){
          latency_mms = computation_latency;// + latency_distribution_micro[rand() % latency_distribution_size];//
          rclcpp::sleep_for(std::chrono::microseconds(latency_mms));
        }

        time_pose_delayed = clk.now();
        // write both time measurments after receiving the pose, to make sure it relates to the delayed pose!
        nano_seconds_pose_sensed = time_pose_sensed.nanoseconds();
        nano_seconds_pose_delayed = time_pose_delayed.nanoseconds();
        //seconds_pose_delayed = time_pose_delayed.seconds();

        //RCLCPP_INFO(this->get_logger(), "delayed stamp set: %f", time_pose_delayed.seconds());
        get_model_state_pose_delayed = get_model_state_pose;
        //get_model_state_euler_delayed = ToEulerAngles(get_model_state_pose_delayed.orientation);
        //RCLCPP_INFO(this->get_logger(), "delayed pose set");
        //RCLCPP_INFO(this->get_logger(), "pose set, x: %f, y: %f, yaw: %f", get_model_state_pose.position.x, get_model_state_pose.position.y, get_model_state_euler.yaw);
        //history_gazebo.push_back({seconds_pose_sensed,seconds_pose_delayed,get_model_state_pose.position.x,get_model_state_pose.position.y,get_model_state_euler.yaw});
      }

      message_.pose = get_model_state_pose_delayed;
      message_.latency_mms = latency_mms;
      message_.time_stamp_origin = nano_seconds_pose_sensed;
      message_.time_stamp_delayed = nano_seconds_pose_delayed;
      message_.lost_packets = lost_packets;
      pose_publisher_->publish(message_);
    }

    /*void get_delayed_pose(std::shared_ptr<custom_interfaces::srv::PoseSensing::Request>  req,
             std::shared_ptr<custom_interfaces::srv::PoseSensing::Response> res)
    {
      res->pose = get_model_state_pose_delayed;
      res->latency_mms = latency_mms;
      res->time_stamp_origin = nano_seconds_pose_sensed;
      res->time_stamp_delayed = nano_seconds_pose_delayed;

    }*/

    void set_pose_sesing_settings(std::shared_ptr<custom_interfaces::srv::PoseSensingSettings::Request>  req,
             std::shared_ptr<custom_interfaces::srv::PoseSensingSettings::Response> res)
    {
      network = req->network;
      cube_id = req->objid;
      computation_latency = req->computation_ms*1000; //in micro seconds
      n_freshness_samples = req->n_freshness_samples;
      framerate = req->sensing_rate;
      qos_profile_key = req->qos_profile;
      frame_rate_microsec = (float)1/framerate*1000*1000;

      // Select the QoS profile based on the key
      if (qos_profile_key == "B10") {
          selected_qos_profile = qos_profile_b10;
      } else if (qos_profile_key == "B1") {
          selected_qos_profile = qos_profile_b1;
      } else if (qos_profile_key == "R10") {
          selected_qos_profile = qos_profile_r10;
      } else if (qos_profile_key == "R1") {
          selected_qos_profile = qos_profile_r1;
      } else {
          RCLCPP_WARN(this->get_logger(), "Invalid QoS profile key. Using the default R10.");
      }


      std::string network_filename = network_directory + network.c_str() + ".csv";
      latency_distribution.clear();
      std::ifstream  data(network_filename);
      if(data.is_open())
      {
        std::string line;
        while(std::getline(data, line)){
          latency_distribution.emplace_back(std::stold(line));
        }
      }
      data.close();
      latency_distribution_size = (int)latency_distribution.size();
      if(latency_distribution_size==0){
        RCLCPP_ERROR(this->get_logger(), "Network data not found at %s, continuing without network latency", network_filename.c_str());
        if (computation_latency == 0){set_delay_mode = 0;}
        else {set_delay_mode = 3;}
      }
      else {
        if (computation_latency == 0){set_delay_mode = 1;}
        else {set_delay_mode = 2;}
        RCLCPP_INFO(this->get_logger(), "Latency distribution found at: %s with size: %d", network_filename.c_str(),(int)latency_distribution.size());

      }
      RCLCPP_INFO(this->get_logger(), "New settings with delay mode: %d", set_delay_mode);
      //RCLCPP_INFO(this->get_logger(), "Starting timer_callback that requests pose from gazebo, delays it and publishes the result as /pose_sensing/delayed_pose ...");
      pose_publisher_ = this->create_publisher<custom_interfaces::msg::PoseSensing>("/pose_sensing/delayed_pose", selected_qos_profile); //R10 is the default, 10 is the queue size
      timer_ptr_ = this->create_wall_timer(std::chrono::microseconds(frame_rate_microsec), std::bind(&GazeboUpdateNode::timer_callback, this),timer_cb_group_);

      res->delay_mode = set_delay_mode;
      res->success = 1;

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto gazebo_update_node = std::make_shared<GazeboUpdateNode>();
  //auto push_node = std::make_shared<push_edge_main::RobotPushNode>();
  //auto robot_setup_node = std::make_shared<push_experiment_02::RobotSetupNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(gazebo_update_node);
  //executor.add_node(push_node);

  executor.spin();


  rclcpp::shutdown();
  return 0;
}
