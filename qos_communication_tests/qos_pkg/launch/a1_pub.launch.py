import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='qos_pkg',
            executable='poses_a1_pub',
            name='poses_a1_pub'),
            parameters=[{'RMW_IMPLEMENTATION':= 'rmw_fastrtps_cpp', 'ros__parameters':= {'discovery_server.address':= '192.168.1.100'}}],
  ])
