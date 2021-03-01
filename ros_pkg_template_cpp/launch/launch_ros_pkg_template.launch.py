from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_node = os.path.join(
        get_package_share_directory('status_node'),
        'config',
        'param' + '.yaml'
    )
    return LaunchDescription([
        Node(
            package='ros_pkg_template',
            namespace='ns',
            executable='ros_pkg_template_node',
            name='ros_pkg_template_name',
            parameters=[
                config_node,
            ],
            remappings=[
                ('input_float_topic', '/ns/ros_pkg_template_node/input'),
                ('output_float_topic', '/ns/ros_pkg_template_node/output'),
            ]
        )
    ])