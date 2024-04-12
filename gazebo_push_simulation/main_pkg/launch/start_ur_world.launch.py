import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg = get_package_share_directory('main_pkg')
    pkg_gripper = get_package_share_directory('robotiq_2f_model')
    pkg_ur = get_package_share_directory('ur_description')

    # Get the whole install dir
    ur_description_package_name = "ur_description"
    ur_install_dir = get_package_prefix(ur_description_package_name)

    gripper_description_package_name = "robotiq_2f_model"
    gripper_install_dir = get_package_prefix(gripper_description_package_name)

    cube_description_package_name = "main_pkg"
    cube_install_dir = get_package_prefix(cube_description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in the picknplace_env package
    gazebo_models_path = os.path.join(pkg, 'worlds')
    gazebo_models_path_gripper = os.path.join(pkg_gripper, 'models')
    gazebo_models_path_ur = os.path.join(pkg_ur, 'meshes')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + cube_install_dir + '/share' + ':' + os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path + '/share' + ':' + os.environ['GAZEBO_MODEL_PATH'] + ':' + ur_install_dir + '/share' + ':' + os.environ['GAZEBO_MODEL_PATH'] + ':' + gripper_install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  cube_install_dir + "/share" + ':' + gazebo_models_path + "/share" + ':' + ur_install_dir + "/share" + ':' + gripper_install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] + ':' + ur_install_dir + '/lib' + ':' + os.environ['GAZEBO_PLUGIN_PATH'] + ':' + cube_install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = ur_install_dir + '/lib' + ':' + cube_install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg, 'worlds', 'empty_plugin.world'), ''],
          description='SDF world file'),
        gazebo
    ])
