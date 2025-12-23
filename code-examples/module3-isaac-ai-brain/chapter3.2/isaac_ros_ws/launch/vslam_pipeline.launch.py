import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for the nodes'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level for the nodes'
    )

    # VSLAM Node
    vslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam_node',
        output='screen',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'denoise_input_images': False,
            'rectify_input_images': True, # Isaac Sim usually provides rectified images
            'enable_imu_fusion': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'tracking_frame': 'camera_link',
            'input_base_frame': 'camera_link',
            'input_left_camera_info_url': 'package://robot_description/config/left_camera_info.yaml', # Placeholder
            'input_right_camera_info_url': 'package://robot_description/config/right_camera_info.yaml', # Placeholder
            'input_left_image_topic': '/stereo_camera/left/image_raw',
            'input_right_image_topic': '/stereo_camera/right/image_raw',
            'publish_odometry': True,
            'publish_tf': True,
            'publish_pose': True,
            'enable_slam': True,
            'enable_localization': True,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Optional: Image rectification node if images are not already rectified
    # stereo_image_proc_node = Node(
    #     package='stereo_image_proc',
    #     executable='stereo_image_proc',
    #     name='stereo_image_proc',
    #     output='screen',
    #     namespace=namespace,
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ('/stereo_camera/left/image_raw', '/stereo_camera/left/image_raw'),
    #         ('/stereo_camera/right/image_raw', '/stereo_camera/right/image_raw'),
    #         ('/stereo_camera/left/camera_info', '/stereo_camera/left/camera_info'),
    #         ('/stereo_camera/right/camera_info', '/stereo_camera/right/camera_info'),
    #     ]
    # )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_log_level_cmd,
        vslam_node,
        # stereo_image_proc_node # Uncomment if needed
    ])
