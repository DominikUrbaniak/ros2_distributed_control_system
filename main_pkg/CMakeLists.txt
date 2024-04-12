cmake_minimum_required(VERSION 3.8)
project(main_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs)
find_package(tf2_geometry_msgs)
find_package(visualization_msgs)
find_package(custom_interfaces)
find_package(gazebo_msgs)
find_package(tf2_ros)
find_package(tf2)
find_package(Eigen3)
find_package(kinenik)
find_package(sensor_msgs)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    config
    launch
    meshes
    rviz
    src
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  launch/spawn_cube_launch.py
  launch/spawn_cube.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(world_setup src/world_setup.cpp)
ament_target_dependencies(world_setup rclcpp geometry_msgs visualization_msgs gazebo_msgs custom_interfaces tf2_ros tf2 tf2_geometry_msgs)

#add_executable( velocity_controller src/velocity_controller.cpp )
#ament_target_dependencies(velocity_controller rclcpp std_msgs sensor_msgs custom_interfaces tf2 tf2_ros)

add_executable( set_config_client src/set_config_client.cpp )
ament_target_dependencies(set_config_client rclcpp custom_interfaces)

add_executable( cartesian_velocity_controller src/cartesian_velocity_controller.cpp )
ament_target_dependencies(cartesian_velocity_controller rclcpp std_msgs sensor_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros kinenik OpenCV)

add_executable( cartesian_velocity_controller_ur3e src/cartesian_velocity_controller_ur3e.cpp )
ament_target_dependencies(cartesian_velocity_controller_ur3e rclcpp std_msgs sensor_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros kinenik OpenCV)

add_executable( real_cartesian_velocity_controller_ur3e src/real_cartesian_velocity_controller_ur3e.cpp )
ament_target_dependencies(real_cartesian_velocity_controller_ur3e rclcpp std_msgs sensor_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros kinenik OpenCV)

add_executable( inv_jacobian src/inv_jacobian.cpp )
ament_target_dependencies(inv_jacobian rclcpp std_msgs sensor_msgs Eigen3 custom_interfaces geometry_msgs tf2 tf2_ros kinenik OpenCV)

add_executable(pose_sensing src/pose_sensing.cpp)
ament_target_dependencies(pose_sensing rclcpp geometry_msgs visualization_msgs gazebo_msgs custom_interfaces tf2_ros tf2 tf2_geometry_msgs)

#add_executable(cv_marker_generation src/cv_marker_generation.cpp)
#ament_target_dependencies(cv_marker_generation rclcpp sensor_msgs cv_bridge OpenCV custom_interfaces)


install(TARGETS
  world_setup
  #velocity_controller
  set_config_client
  cartesian_velocity_controller
  real_cartesian_velocity_controller_ur3e
  pose_sensing
  inv_jacobian
  #cv_marker_generation
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
