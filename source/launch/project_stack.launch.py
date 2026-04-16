from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('project')
    launch_dir = os.path.join(package_share, 'launch')

    camera_depth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'camera_depth.launch.py')
        ),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
        }.items(),
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gps.launch.py')
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'imu.launch.py')
        ),
        launch_arguments={
            'imu_port': LaunchConfiguration('imu_port'),
            'imu_baud': LaunchConfiguration('imu_baud'),
            'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lidar.launch.py')
        ),
        launch_arguments={
            'lidar_params_file': LaunchConfiguration('lidar_params_file'),
        }.items(),
    )

    rgb_yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rgb_yolo.launch.py')
        ),
        launch_arguments={
            'rgb_yolo_camera_source': LaunchConfiguration('rgb_yolo_camera_source'),
            'rgb_yolo_capture_period_sec': LaunchConfiguration('rgb_yolo_capture_period_sec'),
            'yolo_model_path': LaunchConfiguration('yolo_model_path'),
            'yolo_classes_path': LaunchConfiguration('yolo_classes_path'),
            'yolo_conf_threshold': LaunchConfiguration('yolo_conf_threshold'),
            'photo_output_dir': LaunchConfiguration('photo_output_dir'),
        }.items(),
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.62', '0.0', '0.0',
            '1.0', '0.0', '0.0', '0.0',
            'base_link', 'laser'
        ],
        output='screen',
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '0.62', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'base_link', 'imu_link'
        ],
        output='screen',
    )

    static_tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gps',
        arguments=[
            '-0.68', '0.28', '0.70',
            '0.0', '0.0', '0.0', '1.0',
            'base_link', 'gps'
        ],
        output='screen',
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0.62', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'base_link', 'camera_link'
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('imu_port', default_value='/dev/imu'),
        DeclareLaunchArgument('imu_baud', default_value='460800'),
        DeclareLaunchArgument('imu_rate_hz', default_value='50'),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(package_share, 'config', 'lsn10_serial.yaml'),
        ),
        DeclareLaunchArgument('rgb_yolo_camera_source', default_value='/dev/video0'),
        DeclareLaunchArgument('rgb_yolo_capture_period_sec', default_value='1.0'),
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
        camera_depth_launch,
        gps_launch,
        imu_launch,
        lidar_launch,
        rgb_yolo_launch,
        static_tf_laser,
        static_tf_imu,
        static_tf_gps,
        static_tf_camera,
    ])
