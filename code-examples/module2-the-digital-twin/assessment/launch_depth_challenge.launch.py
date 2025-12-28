from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robotics_book_code')
    
    # Path to a generic world file for spawning the robot with depth camera
    world_path = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.1/worlds/empty.sdf')
    
    # Path to the URDF for Challenge 2 (assuming it's created by the student)
    # This URDF should have a depth camera sensor defined.
    urdf_path = os.path.join(pkg_share, 'module2-the-digital-twin/assessment/humanoid_depth_camera_challenge.urdf.xacro')

    # Process the XACRO file (student would create this)
    # This is a placeholder; student would provide the actual xacro file.
    # For testing, you might need a dummy xacro file.
    robot_description = ""
    if os.path.exists(urdf_path):
        doc = xacro.process_file(urdf_path)
        robot_description = doc.toxml()
    else:
        # Fallback for when the student hasn't created the URDF yet.
        # This part won't run correctly, but prevents launch file failure.
        print(f"WARNING: URDF file not found at {urdf_path}. Please create it for Challenge 2.")

    return LaunchDescription([
        # Example: Start Gazebo (student would integrate this)
        # ExecuteProcess(
        #     cmd=['ign', 'gazebo', '-r', world_path],
        #     output='screen'
        # ),

        # Example: Robot State Publisher (student would integrate this)
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        # ),

        # Example: Spawn robot (student would integrate this)
        # Node(
        #     package='ros_gz_sim',
        #     executable='create',
        #     name='humanoid_challenge2',
        #     arguments=['-topic', 'robot_description', '-name', 'humanoid_challenge2', '-z', '1.0'],
        #     output='screen'
        # ),

        # Node to process depth data (from assessment_depth_processor.py)
        Node(
            package='robotics_book_code',
            executable='depth_processor_challenge.py',
            name='depth_processor_challenge',
            output='screen'
        )
    ])
