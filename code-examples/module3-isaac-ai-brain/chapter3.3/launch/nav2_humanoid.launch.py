import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths to your Nav2 configurations
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # Assuming 'robotics_book_code' is your main package for this book's examples
    book_code_pkg_dir = get_package_share_directory('robotics_book_code')
    
    nav2_config_dir = os.path.join(book_code_pkg_dir, 'module3-isaac-ai-brain', 'chapter3.3', 'nav2_config')
    costmap_params_path = os.path.join(nav2_config_dir, 'costmap_params.yaml')
    planner_params_path = os.path.join(nav2_config_dir, 'planner_params.yaml')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the Nav2 stack'
    )
    robot_base_frame = DeclareLaunchArgument(
        'robot_base_frame', default_value='base_link', description='Robot base frame'
    )
    global_frame = DeclareLaunchArgument(
        'global_frame', default_value='map', description='Global frame (map frame)'
    )

    return LaunchDescription([
        use_sim_time,
        autostart,
        robot_base_frame,
        global_frame,

        # Include the main Nav2 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map_subscribe_transient_local': 'true', # Important for dynamic VSLAM maps
                'params_file': planner_params_path, # Pass our custom planner parameters
                'costmap_params_file': costmap_params_path, # Pass our custom costmap parameters
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'global_frame': LaunchConfiguration('global_frame'),
                # Add other parameters as needed, e.g., for localization or map server
            }.items(),
        ),

        # Note: You would typically launch your Isaac Sim environment and VSLAM pipeline
        # in a separate launch file or include them here.
        # This launch file focuses only on the Nav2 stack itself.
        # Example of how you might include VSLAM:
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(book_code_pkg_dir, 'module3-isaac-ai-brain', 'chapter3.2', 'isaac_ros_ws', 'launch', 'vslam_pipeline.launch.py'))
        # ),
    ])
