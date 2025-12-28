import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    llm_model = LaunchConfiguration('llm_model')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Isaac Sim) clock if true'
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level for all nodes'
    )
    declare_llm_model_cmd = DeclareLaunchArgument(
        'llm_model', default_value='gpt-4-turbo-preview', description='LLM model to use for planning'
    )

    # Paths to capstone packages (assuming they are in the same ROS 2 workspace)
    capstone_pkg_share_dir = get_package_share_directory('autonomous_humanoid_nav') # Example package name

    # Placeholder for Isaac Sim launch (assuming it's running externally or integrated)
    # This might involve launching an Isaac Sim USD scene via 'ign gazebo' or similar,
    # and ensuring ROS 2 bridge is active for sensor data.

    # 1. Voice Command Node
    voice_command_node = Node(
        package='voice_command_node',
        executable='voice_command_node.py',
        name='voice_command_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 2. LLM Planner Node
    llm_planner_node = Node(
        package='llm_planner_node',
        executable='llm_planner_node.py',
        name='llm_planner_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time, 'llm_model': llm_model}],
    )

    # 3. VLA Control Node
    vla_control_node = Node(
        package='vla_control_node',
        executable='vla_control_node.py',
        name='vla_control_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 4. Integrate Nav2 stack (conceptual)
    # This would involve including a Nav2 launch file, similar to Chapter 3.3,
    # configured for your specific humanoid and environment.
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
    #     )),
    #     launch_arguments={
    #         'map_subscribe_transient_local': 'true',
    #         'params_file': os.path.join(capstone_pkg_share_dir, 'config', 'nav2_params.yaml'),
    #         'use_sim_time': use_sim_time,
    #         'autostart': 'true',
    #         'robot_base_frame': 'base_link',
    #         'global_frame': 'map',
    #     }.items(),
    # )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_log_level_cmd,
        declare_llm_model_cmd,
        
        voice_command_node,
        llm_planner_node,
        vla_control_node,
        # nav2_launch, # Uncomment and configure for Nav2 integration
    ])
