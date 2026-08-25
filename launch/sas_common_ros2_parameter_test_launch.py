"""Launch file used to run the ros2 parameter test node for sas_common.

This launch description starts the `sas_common_ros2_parameter_test_node` with
parameters loaded from a YAML configuration file. The parameters exercise the
special EMPTY_LIST handling for vector parameters.

Pass a different file with ``config_file:=/path/to/config.yaml``.
"""

import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the sas_common ROS2 parameter test node.

    Parameters are loaded from a YAML configuration file. Pass a different
    file with ``config_file:=/path/to/config.yaml``.
    """
    name = LaunchConfiguration('name')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='sas_common_ros2_parameter_test'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('sas_common'), 'config', 'config.yaml')
        ),
        Node(
            package='sas_common',
            executable='sas_common_ros2_parameter_test_node',
            name=name,
            output='screen',
            parameters=[config_file]
        )
    ])

