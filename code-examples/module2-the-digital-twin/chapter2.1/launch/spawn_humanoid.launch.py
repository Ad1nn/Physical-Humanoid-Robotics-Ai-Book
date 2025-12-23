import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robotics_book_code')
    world_path = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.1/worlds/empty.sdf')
    xacro_file = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.1/urdf/humanoid_ch2.1.urdf.xacro')

    # Process the XACRO file to generate the URDF
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    # Start Gazebo with the specified world
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    # Node to publish the robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_humanoid',
        arguments=['-topic', 'robot_description', '-name', 'humanoid', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot
    ])
