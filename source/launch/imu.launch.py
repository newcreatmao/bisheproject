from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_prefix = get_package_prefix('project')
    imu_share = get_package_share_directory('yesense_std_ros2')
    imu_setup_executable = os.path.join(package_prefix, 'lib', 'project', 'configure_h30_imu.py')
    yesense_params_file = os.path.join(imu_share, 'config', 'yesense_config.yaml')

    imu_setup_process = ExecuteProcess(
        cmd=[
            imu_setup_executable,
            '--port', LaunchConfiguration('imu_port'),
            '--baud', LaunchConfiguration('imu_baud'),
            '--rate-hz', LaunchConfiguration('imu_rate_hz'),
        ],
        output='screen',
    )

    yesense_process = Node(
        package='yesense_std_ros2',
        executable='yesense_node_publisher',
        name='yesense_pub',
        parameters=[
            yesense_params_file,
            {
                'serial_port': LaunchConfiguration('imu_port'),
                'baud_rate': ParameterValue(LaunchConfiguration('imu_baud'), value_type=int),
            },
        ],
        respawn=True,
        respawn_delay=2.0,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value='/dev/imu'),
        DeclareLaunchArgument('imu_baud', default_value='460800'),
        DeclareLaunchArgument('imu_rate_hz', default_value='50'),
        imu_setup_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_setup_process,
                on_exit=[yesense_process],
            )
        ),
    ])
