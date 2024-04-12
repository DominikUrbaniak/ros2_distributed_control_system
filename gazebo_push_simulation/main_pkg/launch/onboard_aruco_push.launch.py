import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    '''return LaunchDescription([
        Node(
            package='push_control_py',
            executable='a1_image_publisher',
            name='a1_image_publisher',
            output='screen',
        ),
        Node(
            package='push_control_py',
            executable='a2_aruco_detection',
            name='a2_aruco_detection',
            output='screen',
        ),
        Node(
            package='push_control_py',
            executable='real_pose_sim_push',
            name='real_pose_sim_push',
            output='screen',
        ),
    ])'''
