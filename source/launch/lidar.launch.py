from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('project')
    default_lidar_params_file = os.path.join(package_share, 'config', 'lsn10_serial.yaml')

    lidar_process = Node(
        package='project',
        executable='lsn10_serial_scan_node',
        name='lslidar_driver_node',
        namespace='',
        parameters=[LaunchConfiguration('lidar_params_file')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_params_file', default_value=default_lidar_params_file),
        lidar_process,
    ])
