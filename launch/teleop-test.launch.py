import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mqtt_teleop')
    cfg = os.path.join(pkg_share, 'config', 'teleop.yaml')
    urdf_file = os.path.join(pkg_share, 'config', 'dummy.urdf.xml')

    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='right', description='right | left'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='agx_arm_ctrl の namespace (左右2台なら right / left など)'),
        DeclareLaunchArgument('broker_uri', default_value='wss://127.0.0.1:8333'),
        DeclareLaunchArgument('liu', default_value='false'),
        Node(
            package='mqtt_teleop',
            executable='teleop_node',
            name='mqtt_teleop',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[cfg, {
                'arm': LaunchConfiguration('arm'),
                'broker_uri': LaunchConfiguration('broker_uri'),
                'liu': ParameterValue(LaunchConfiguration('liu'), value_type=bool),
                'feedback_timeout_s': 0.6,
            }],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            # arguments 引数を使って、ダミーのURDFファイルを直接流し込みます
            arguments=[urdf_file],
            parameters=[{
                'rate': 2  # 2Hz
            }],
            remappings=[
                ('joint_states', 'feedback/joint_states')
            ]
        ),
    ])
