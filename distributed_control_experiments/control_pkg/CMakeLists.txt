cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

install(
  DIRECTORY
    #launch
    config
    src
  DESTINATION
    share/${PROJECT_NAME}/
)

add_executable( real_cartesian_velocity_controller_ur3e src/real_cartesian_velocity_controller_ur3e.cpp )
ament_target_dependencies(real_cartesian_velocity_controller_ur3e rclcpp std_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros)

add_executable( rcvc_sub_ur3e src/rcvc_sub_ur3e.cpp )
ament_target_dependencies(rcvc_sub_ur3e rclcpp std_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros)

add_executable( rcvc_sub_ur5e src/rcvc_sub_ur5e.cpp )
ament_target_dependencies(rcvc_sub_ur5e rclcpp std_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros)

install(TARGETS
  real_cartesian_velocity_controller_ur3e
  rcvc_sub_ur3e
  rcvc_sub_ur5e
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
