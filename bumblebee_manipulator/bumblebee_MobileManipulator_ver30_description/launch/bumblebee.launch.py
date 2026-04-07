from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 패키지 경로
    bumblebee_pkg = get_package_share_directory('bumblebee_MobileManipulator_ver30_description')
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')
    zed_wrapper_pkg = get_package_share_directory('zed_wrapper')

    
    xacro_path = os.path.join(bumblebee_pkg, 'urdf', 'bumblebee_MobileManipulator_ver4.xacro')
    robot_description_config = xacro.process_file(xacro_path)
    robot_urdf = robot_description_config.toxml()

    # 로봇 상태 퍼블리셔
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # ZED 카메라 TF 고정
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_tf_pub',
        arguments=['0', '0', '0.34', '0', '0', '0', 'base_link', 'zed_camera_link'],
        output='screen'
    )

    # ZED 카메라 런치 포함
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_pkg, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2',
            'base_frame': 'zed_camera_link',
            'publish_tf': 'false'
        }.items()
    )

    # RTAB-Map 런치 포함
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_pkg, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'stereo': 'false',
            'localization': LaunchConfiguration('localization'),
            'rtabmap_viz': 'true',
            'rviz': 'true',
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'rgb_topic': '/zed/zed_node/rgb/image_rect_color',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/rgb/camera_info',
            'imu_topic': '/zed/zed_node/imu/data',
            'database_path': LaunchConfiguration('database_path'),
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database for localization.'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode (true) or mapping mode (false)'
        ),
        joint_state_publisher_gui_node,
        robot_state_pub_node,
        static_tf_pub_node,
        zed_launch,
        rtabmap_launch
    ])