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
    ar_detection_pkg = 'team12_ar_detection_pkg'
    ar_node_name = 'ar_detection_node'

    ld = LaunchDescription()
    
    ar_detection_node = Node(
        package=ar_detection_pkg,
        executable=ar_node_name,
        output='screen',
        parameters=[])
        
    ld.add_action(ar_detection_node)
    return ld
