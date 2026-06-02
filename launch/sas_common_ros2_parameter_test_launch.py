"""Launch file used to run the ros2 parameter test node for sas_common.

This launch description starts the `sas_common_ros2_parameter_test_node` with
pre-configured parameters used to exercise the special EMPTY_LIST handling for
vector parameters.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_common',
            executable='sas_common_ros2_parameter_test_node',
            name='sas_common_ros2_parameter_test',
            output='screen',
            parameters=[{
                "empty_string_vector": ["EMPTY_LIST"],
                "empty_integer_vector":  ["EMPTY_LIST"],
                "empty_double_vector":  ["EMPTY_LIST"],
                "empty_bool_vector":  ["EMPTY_LIST"],
                "string_vector": ["a","b","c","d","e","f"],
                "integer_vector":  [1,2,3,4,5,6,7,8,9,10],
                "double_vector":  [11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0],
                "bool_vector":  [False,True,True,False,True,True,True,True,False,True,True,False,True,False,True,True],
            }]
        )
    ])

