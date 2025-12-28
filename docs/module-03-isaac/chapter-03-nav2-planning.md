---
sidebar_position: 4
title: '3.3: Nav2 Path Planning'
---

# Chapter 3.3: Nav2 Path Planning

With the ability to simulate humanoid robots in Isaac Sim and localize them using Isaac ROS VSLAM, our final step is to enable **autonomous navigation**. This chapter delves into the **Nav2** navigation stack in ROS 2, adapting it for the unique challenges of bipedal locomotion, enabling your humanoid to plan paths and avoid obstacles.

## 1. Nav2 Navigation Stack Overview

Nav2 is the standard ROS 2 solution for autonomous navigation. It provides a modular and extensible framework that covers:

-   **Localization**: Determining the robot's pose within a map (which we get from VSLAM).
-   **Path Planning**: Generating a global path from the robot's current location to a goal.
-   **Local Planning/Control**: Executing the global path while avoiding dynamic obstacles.
-   **Recovery Behaviors**: Handling situations where the robot gets stuck or deviates from its path.

## 2. Bipedal vs. Wheeled Robot Navigation Challenges

Nav2 was originally designed primarily for wheeled robots. Humanoid robots introduce specific challenges:

-   **Kinematics**: Bipedal robots have complex, high-dimensional kinematics.
-   **Dynamics**: Walking involves dynamic balance, making simple velocity commands insufficient.
-   **Footstep Planning**: Navigation often involves discrete footstep planning rather than continuous path following.
-   **Costmaps**: Costmap configuration needs to account for the robot's shape and walking gait.

Adapting Nav2 for humanoids often involves custom plugins or carefully tuned parameters, as direct support is still an active research area.

## 3. Costmap Configuration for Humanoid Robots

Costmaps are grid-based representations of the environment that Nav2 uses for obstacle avoidance and path planning. They assign a "cost" to each cell, indicating how difficult or dangerous it is for the robot to occupy that space.

For humanoids, costmaps need special consideration:

-   **Robot Footprint**: The `footprint` parameter should reflect the actual area covered by the humanoid while walking, not just its base.
-   **Inflation Layer**: How much to inflate obstacles to give the robot clearance, accounting for swinging arms/legs.
-   **Dynamic Obstacles**: How quickly the costmap updates to reflect moving obstacles.

**File: `code-examples/module3-isaac-ai-brain/chapter3.3/nav2_config/costmap_params.yaml`**
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: true
      use_sim_time: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_topic: "/map" # Map from VSLAM
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: "scan cloud" # LiDAR and Depth Camera
        scan:
          topic: "/lidar_scan" # From Isaac Sim LiDAR
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          data_type: "LaserScan"
          clearing: true
          marking: true
          is_lidar: true
          raytrace_range: 3.0
          obstacle_range: 2.5
        cloud:
          topic: "/stereo_camera/points" # From Isaac Sim depth camera
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          data_type: "PointCloud2"
          clearing: true
          marking: true
          raytrace_range: 3.0
          obstacle_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0 # Increase for more aggressive inflation
        inflation_radius: 0.5 # Radius around obstacles

local_costmap:
  local_costmap:
    ros__parameters:
      track_unknown_space: true
      use_sim_time: true
      plugins: ["obstacle_layer", "inflation_layer"]
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 5.0
      height: 5.0
      resolution: 0.05
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: "scan cloud"
        scan:
          topic: "/lidar_scan"
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          data_type: "LaserScan"
          clearing: true
          marking: true
          is_lidar: true
          raytrace_range: 3.0
          obstacle_range: 2.5
        cloud:
          topic: "/stereo_camera/points"
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          data_type: "PointCloud2"
          clearing: true
          marking: true
          raytrace_range: 3.0
          obstacle_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
```

## 4. Path Planning Algorithms

Nav2 offers several global and local planners. For humanoid navigation, the choice and tuning are crucial.

-   **Global Planners**: Compute a path from start to goal through the entire map (e.g., `NavFn`, `SmacPlanner`).
-   **Local Planners**: Execute the global path segment by segment while avoiding dynamic obstacles and respecting robot kinematics (e.g., DWA, TEB, Regulated Pure Pursuit).

**File: `code-examples/module3-isaac-ai-brain/chapter3.3/nav2_config/planner_params.yaml`**
```yaml
planner_server:
  ros__parameters:
    use_sim_time: true
    global_costmap_topic: /global_costmap/costmap_raw
    local_costmap_topic: /local_costmap/costmap_raw
    plugins: ["GridBased", "SmacHybrid", "SmacLattice"] # Example global planners
    SmacHybrid:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      cost_travel_multiplier: 1.0
      downsample_costmap_ratio: 1
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.25

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"] # Example local controller
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5 # For a walking humanoid
      min_speed_xy: 0.1
      max_robot_pose_update_frequency: 50.0
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      transform_tolerance: 0.1
      max_allowed_time_to_collision: 1.0 # Avoid immediate collisions

bt_navigator:
  ros__parameters:
    use_sim_time: true
    default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
```

## 5. Integrating Nav2 for Humanoid Navigation

The full Nav2 stack for a humanoid would typically be launched via a single Python launch file, integrating:

-   Map Server (loading map from VSLAM)
-   AMCL (or VSLAM pose for localization)
-   Costmap Filters
-   Global and Local Planners
-   Controller Server
-   Behavior Tree Navigator

**File: `code-examples/module3-isaac-ai-brain/chapter3.3/launch/nav2_humanoid.launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Set environment variables for Nav2
    # Ensure correct paths to your Nav2 configurations
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # Assuming 'robotics_book_code' is your main package for this book's examples
    book_code_pkg_dir = get_package_share_directory('robotics_book_code')
    
    nav2_config_dir = os.path.join(book_code_pkg_dir, 'module3-isaac-ai-brain', 'chapter3.3', 'nav2_config')
    costmap_params_path = os.path.join(nav2_config_dir, 'costmap_params.yaml')
    planner_params_path = os.path.join(nav2_config_dir, 'planner_params.yaml')

    return LaunchDescription([
        # Set use_sim_time for Gazebo/Isaac Sim
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Nav2 itself
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map_subscribe_transient_local': 'true', # Important for dynamic VSLAM maps
                'params_file': planner_params_path,
                'costmap_params_file': costmap_params_path, # Pass our custom costmap parameters
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
                'robot_base_frame': 'base_link',
                'global_frame': 'map',
            }.items(),
        ),

        # For costmaps, you might need to launch your own costmap nodes or ensure
        # that the Nav2 bringup parameters file correctly points to your custom costmap_params.yaml
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[costmap_params_path],
            arguments=['--ros-args', '-r', '__ns:=/global_costmap']
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[costmap_params_path],
            arguments=['--ros-args', '-r', '__ns:=/local_costmap']
        ),
        
        # You would also need to run your VSLAM to provide the map and localization.
        # Example: IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('isaac_ros_visual_slam'), 'launch', 'visual_slam_realsense.launch.py'))
        # )
    ])
```

## Summary

This chapter has guided you through configuring the Nav2 stack for humanoid autonomous navigation. You've learned about costmap tuning, path planning algorithms, and how to integrate Nav2 into your Isaac Sim environment, bringing your humanoid robot closer to truly intelligent behavior.
