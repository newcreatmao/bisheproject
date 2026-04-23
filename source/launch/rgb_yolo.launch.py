from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('project')

    rgb_yolo_process = Node(
        package='project',
        executable='rgb_yolo_detector',
        name='rgb_yolo_detector',
        parameters=[{
            'camera_source': LaunchConfiguration('rgb_yolo_camera_source'),
            'capture_period_sec': LaunchConfiguration('rgb_yolo_capture_period_sec'),
            'yolo_model_path': LaunchConfiguration('yolo_model_path'),
            'yolo_classes_path': LaunchConfiguration('yolo_classes_path'),
            'yolo_conf_threshold': LaunchConfiguration('yolo_conf_threshold'),
            'photo_output_dir': LaunchConfiguration('photo_output_dir'),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rgb_yolo_camera_source', default_value='/dev/video0'),
        DeclareLaunchArgument('rgb_yolo_capture_period_sec', default_value='0.333'),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=os.path.join(package_share, 'pt', 'best.onnx'),
        ),
        DeclareLaunchArgument(
            'yolo_classes_path',
            default_value=os.path.join(package_share, 'pt', 'classes.txt'),
        ),
        DeclareLaunchArgument('yolo_conf_threshold', default_value='0.35'),
        DeclareLaunchArgument(
            'photo_output_dir',
            default_value='~/use/project/source/allfile/photos',
        ),
        rgb_yolo_process,
    ])
