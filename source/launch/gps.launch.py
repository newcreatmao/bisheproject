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
    package_share = get_package_share_directory('project')
    gps_setup_executable = os.path.join(package_prefix, 'lib', 'project', 'configure_g60_gps.py')
    default_gps_params_file = os.path.join(package_share, 'config', 'g60_nmea_driver.yaml')

    gps_setup_process = ExecuteProcess(
        cmd=[
            gps_setup_executable,
            '--port', LaunchConfiguration('gps_port'),
            '--probe-baud', LaunchConfiguration('gps_initial_baud'),
            '--probe-baud', LaunchConfiguration('gps_baud'),
            '--target-baud', LaunchConfiguration('gps_baud'),
            '--rate-hz', LaunchConfiguration('gps_rate_hz'),
            '--verify-seconds', LaunchConfiguration('gps_verify_seconds'),
        ],
        output='screen',
    )

    gps_driver_process = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_navsat_driver',
        parameters=[
            LaunchConfiguration('gps_params_file'),
            {
                'port': LaunchConfiguration('gps_port'),
                'baud': ParameterValue(LaunchConfiguration('gps_baud'), value_type=int),
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gps_params_file', default_value=default_gps_params_file),
        DeclareLaunchArgument('gps_port', default_value='/dev/gps'),
        DeclareLaunchArgument('gps_initial_baud', default_value='9600'),
        DeclareLaunchArgument('gps_baud', default_value='115200'),
        DeclareLaunchArgument('gps_rate_hz', default_value='10'),
        DeclareLaunchArgument('gps_verify_seconds', default_value='1.4'),
        gps_setup_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gps_setup_process,
                on_exit=[gps_driver_process],
            )
        ),
    ])
