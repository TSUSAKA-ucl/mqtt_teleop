import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mqtt_teleop'), 'config', 'teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='right', description='right | left'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='agx_arm_ctrl の namespace (左右2台なら right / left など)'),
        DeclareLaunchArgument('broker_uri', default_value='wss://localhost:8333'),
        DeclareLaunchArgument('liu', default_value='false'),
        Node(
            package='mqtt_teleop',
            executable='teleop_node',
            name='mqtt_teleop',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[cfg, {
                'arm': LaunchConfiguration('arm'),
                'broker_uri': LaunchConfiguration('broker_uri'),
                'liu': ParameterValue(LaunchConfiguration('liu'), value_type=bool),
            }],
        ),
    ])
