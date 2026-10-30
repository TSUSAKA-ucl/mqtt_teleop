import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from venv_utils.venv_utils import get_venv_pythonpath_if_has_pyagxarm


def generate_launch_description():
    node_env = os.environ.copy()
    node_env['PYTHONPATH'] = get_venv_pythonpath_if_has_pyagxarm()

    # 1. 呼び出す各パッケージのlaunchファイルパスを取得
    # agx_arm_ctrl_dir = get_package_share_directory('agx_arm_ctrl')
    agx_arm_ctrl_dir = get_package_share_directory('mqtt_teleop')
    mqtt_teleop_dir = get_package_share_directory('mqtt_teleop')

    start_single_agx_arm_launch = os.path.join(
        agx_arm_ctrl_dir, 'launch', 'start_single_agx_arm.launch.py'
    )
    teleop_launch = os.path.join(
        mqtt_teleop_dir, 'launch', 'teleop.launch.py'
    )

    # 2. 親ファイル側で一元管理する引数の定義（子ファイルのデフォルト値は無視されます）
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for both agx_arm_ctrl and mqtt_teleop'
    )

    # 両方のファイルで強制的に同期させる「左右」の共通引数
    # これにより、それぞれのデフォルト値（right/leftの不一致）は上書きされます
    arm_arg = DeclareLaunchArgument(
        'arm',
        default_value='right',
        choices=['right', 'left'],
        description='Common arm side for both launch files (right | left)'
    )

    # 3. IncludeLaunchDescription を使って各ファイルを呼び出し、値を強制適用
    include_agx_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(start_single_agx_arm_launch),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'revo2_type': LaunchConfiguration('arm'),  # agx_arm_ctrl の revo2_type に親の arm を適用
        }.items()
    )

    include_mqtt_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'arm': LaunchConfiguration('arm'),         # mqtt_teleop の arm に親の arm を適用
        }.items()
    )

    return LaunchDescription([
        namespace_arg,
        arm_arg,
        include_agx_arm,
        include_mqtt_teleop,
    ])
