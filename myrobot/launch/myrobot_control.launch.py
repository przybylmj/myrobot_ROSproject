import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  
        return None


def generate_launch_description():
    
    #load model and config files
    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory("myrobot"),"model","robot_model.urdf",))
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file("myrobot", "model/myrobot.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = load_yaml("myrobot", "model/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    myrobot_move_group = Node(
        name="myrobot_control",
        package="myrobot",
        executable="myrobot_control",
        output="screen",
        parameters=[robot_description,robot_description_semantic,kinematics_yaml]
    )

    return LaunchDescription([myrobot_move_group])
