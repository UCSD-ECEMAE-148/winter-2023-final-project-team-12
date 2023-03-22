import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


def generate_launch_description():
    controller_pkg = 'team12_controller_pkg'
    control_node_name = 'controller_node'
    config_file = 'controller_config.yaml'

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(controller_pkg),
        'config',
        config_file)
    
    controller_node = Node(
        package=controller_pkg,
        executable=control_node_name,
        output='screen',
        parameters=[config])
        
    ld.add_action(controller_node)
    return ld
