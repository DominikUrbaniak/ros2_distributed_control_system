#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "custom_interfaces/srv/rob_conf.hpp"


using namespace std::chrono_literals;

int main( int argc, char** argv )
{
  //(void) argc;
  //(void) argv;
  rclcpp::init(argc, argv);
  //rclcpp::Node node("setting_up");
  if (argc != 8) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: set_config_client ur_joint_config1 ur_joint_config2 ur_joint_config3 ur_joint_config4 ur_joint_config5 ur_joint_config6, gripper_joint");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_config");

  rclcpp::Client<custom_interfaces::srv::RobConf>::SharedPtr client =
    node->create_client<custom_interfaces::srv::RobConf>("/velocity_controller/set_desired_joint_config");

  auto request = std::make_shared<custom_interfaces::srv::RobConf::Request>();
  request->conf = {(float)atof(argv[1]),(float)atof(argv[2]),(float)atof(argv[3]),(float)atof(argv[4]),(float)atof(argv[5]),(float)atof(argv[6]),(float)atof(argv[7])};

  //auto publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(topic, 1000);
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }



  //auto result = std::make_shared<gazebo_msgs::srv::GetEntityState::Response>();


  auto result = client->async_send_request(request);
        // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success: %d", result.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_desired_joint_config");
  }


  rclcpp::shutdown();
  //while(rclcpp::ok());

  return 0;
}
