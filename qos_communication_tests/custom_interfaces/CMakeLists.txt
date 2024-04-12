cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ik.msg"
  "msg/PoseArray.msg"
  "msg/PoseSensing.msg"
  "msg/PosesStampId.msg"
  "msg/ImageStampId.msg"
  "msg/CompressedImageStampId.msg"
  "msg/PoseCommunication.msg"
  "msg/Sync.msg"
  "srv/Urik.srv"
  "srv/ObjPose.srv"
  "srv/RobConf.srv"
  "srv/SetObjId.srv"
  "srv/RobPose.srv"
  "srv/SetRandomization.srv"
  "srv/PoseSensingSettings.srv"
  "srv/ResetPoses.srv"
  "srv/TimeMeasurement.srv"
  "srv/ImageStampIdSrv.srv"
  "srv/PoseDetection.srv"
  "srv/String.srv"
  "srv/QosSettings.srv"
  "srv/PosesStampIdSrv.srv"

  DEPENDENCIES geometry_msgs sensor_msgs

 )

ament_package()
