from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    camera_share = get_package_share_directory('orbbec_camera')

    camera_process = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, 'launch', 'astra.launch.py')
        ),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'enable_color': 'false',
            'enable_depth': 'true',
            'enable_ir': 'false',
            'depth_width': '320',
            'depth_height': '240',
            'depth_fps': '30',
            'depth_format': 'Y11',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'publish_tf': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        camera_process,
    ])
