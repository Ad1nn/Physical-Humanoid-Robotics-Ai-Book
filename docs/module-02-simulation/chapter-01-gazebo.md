---
sidebar_position: 2
title: '2.1: Gazebo Physics Simulation'
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 2.1: Gazebo Physics Simulation

In this chapter, you'll take your first steps into the world of robotics simulation. We'll use **Gazebo**, the most widely-used simulation tool in the ROS ecosystem, to create a virtual world, spawn our humanoid robot, and make it move.

## 1. Introduction to Gazebo

Gazebo is more than just a 3D renderer; it's a powerful physics simulator. It allows us to model:

-   **Rigid Body Dynamics**: How objects move and interact under forces and torques.
-   **Collision Detection**: How objects contact each other.
-   **Sensor Simulation**: Generating realistic data from virtual sensors like cameras and LiDAR.
-   **Actuator Simulation**: Modeling the behavior of motors and joints.

We will be using **Gazebo Garden**, a modern version of the simulator.

## 2. World Files (.sdf)

A Gazebo simulation starts with a "world" file, which uses the **Simulation Description Format (SDF)**. This XML-based file defines everything in the environment: the ground, lighting, physics properties, and the models to be included.

Let's create a simple, empty world.

**File: `code-examples/module2-the-digital-twin/chapter2.1/worlds/empty.sdf`**
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="empty">
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <!-- World lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

This file sets up the physics engine, a sun for lighting, and a ground plane.

## 3. Spawning the Robot

To get our robot into the Gazebo world, we need a ROS 2 **launch file**. This Python script will start the Gazebo simulator and use the `ros_gz_sim` package to spawn our robot model from its URDF file.

Our URDF for this chapter will be a simple version of the humanoid, including the base, torso, and head.

**File: `code-examples/module2-the-digital-twin/chapter2.1/urdf/humanoid_ch2.1.urdf.xacro`**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_ch2.1">
    <!-- Include the base humanoid model -->
    <xacro:include filename="$(find robotics_book_code)/module2-the-digital-twin/shared/humanoid_base.urdf.xacro" />
</robot>
```
*(Note: We assume a ROS 2 package `robotics_book_code` is created to locate files)*

Now, the launch file to bring it all together.

**File: `code-examples/module2-the-digital-twin/chapter2.1/launch/spawn_humanoid.launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robotics_book_code')
    world_path = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.1/worlds/empty.sdf')
    urdf_path = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.1/urdf/humanoid_ch2.1.urdf.xacro')

    return LaunchDescription([
        # Start Gazebo with the specified world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
            output='screen'
        ),

        # Node to convert XACRO to URDF and publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_humanoid',
            arguments=['-topic', 'robot_description', '-name', 'humanoid'],
            output='screen'
        )
    ])
```

To run this, you would build your ROS 2 workspace and execute:
`ros2 launch robotics_book_code spawn_humanoid.launch.py`

## 4. Controlling Joints

Finally, let's write a simple ROS 2 node to publish joint commands. This script will send messages to a topic that `gz_ros2_control` (which we will set up in the URDF later) listens to. For now, we'll just publish the messages.

**File: `code-examples/module2-the-digital-twin/chapter2.1/scripts/joint_controller.py`**
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_command)
        self.get_logger().info('Joint Controller started.')

    def publish_command(self):
        msg = JointTrajectory()
        msg.joint_names = ['torso_to_right_arm']
        point = JointTrajectoryPoint()
        point.positions = [math.sin(self.get_clock().now().nanoseconds / 1e9)] # Simple oscillating motion
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing command: {point.positions[0]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This completes the first user story. You now have a simulated robot in a world and a way to send it commands.