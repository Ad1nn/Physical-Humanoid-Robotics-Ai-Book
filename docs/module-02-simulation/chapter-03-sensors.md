---
sidebar_position: 4
title: '2.3: Sensor Simulation'
---

# Chapter 2.3: Sensor Simulation

For a robot to interact intelligently with its environment, it needs to perceive it. This is where **sensors** come in. In simulation, we can easily add various types of virtual sensors to our robot and generate synthetic data, which is invaluable for developing and testing perception algorithms without needing physical hardware.

## 1. Overview of Robotic Sensors

Humanoid robots, like their biological counterparts, rely on a rich array of sensory inputs:

-   **LiDAR (Light Detection and Ranging)**: Provides 2D or 3D point cloud data, useful for mapping and obstacle avoidance.
-   **Depth Cameras (RGB-D)**: Combine a standard color image with per-pixel depth information.
-   **IMU (Inertial Measurement Unit)**: Measures orientation, angular velocity, and linear acceleration, crucial for balance and navigation.
-   **RGB Cameras**: Standard cameras for visual perception tasks.

In simulation, these sensors generate ROS 2 messages (`sensor_msgs`) that mimic their real-world counterparts, often with configurable noise models.

## 2. Adding a LiDAR Sensor to the Humanoid

We'll extend our base humanoid URDF to include a simulated LiDAR sensor. Gazebo provides plugins for many common sensors.

**File: `code-examples/module2-the-digital-twin/chapter2.3/urdf/humanoid_sensors.urdf.xacro`**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_sensors">

    <!-- Include the base humanoid model -->
    <xacro:include filename="$(find robotics_book_code)/module2-the-digital-twin/shared/humanoid_base.urdf.xacro" />

    <!-- Define a new link for the LiDAR sensor -->
    <link name="lidar_link">
      <visual>
        <geometry>
          <cylinder radius="0.03" length="0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.03" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <!-- Attach the LiDAR to the head link -->
    <joint name="head_to_lidar" type="fixed">
      <parent link="head"/>
      <child link="lidar_link"/>
      <origin xyz="0 0 0.17" rpy="0 0 0"/> <!-- Positioned on top of the head -->
    </joint>

    <!-- Gazebo sensor plugin for LiDAR -->
    <gazebo reference="lidar_link">
      <sensor name="lidar_sensor" type="gpu_lidar">
        <always_on>true</always_on>
        <update_rate>10.0</update_rate>
        <visualize>true</visualize>
        <topic>lidar_scan</topic>
        <ray>
          <scan>
            <horizontal>
              <samples>640</samples>
              <resolution>1</resolution>
              <min_angle>-2.2</min_angle>
              <max_angle>2.2</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="ignition::gazebo::sensors::GpuLidar" filename="ignition-gazebo-sensors">
          <topic>lidar_scan</topic>
          <frame_name>lidar_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>

</robot>
```

This XACRO file adds a `lidar_link` to the top of the robot's head and attaches a `gpu_lidar` sensor to it via a Gazebo plugin. The sensor will publish `sensor_msgs/msg/LaserScan` messages on the `lidar_scan` topic.

## 3. Launching the Sensor-Equipped Robot

Our launch file will now need to bring up Gazebo with our new sensor URDF.

**File: `code-examples/module2-the-digital-twin/chapter2.3/launch/spawn_humanoid_with_sensors.launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robotics_book_code')
    world_path = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.1/worlds/empty.sdf') # Re-using empty world
    xacro_file = os.path.join(pkg_share, 'module2-the-digital-twin/chapter2.3/urdf/humanoid_sensors.urdf.xacro')

    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_humanoid',
            arguments=['-topic', 'robot_description', '-name', 'humanoid', '-z', '1.0'],
            output='screen'
        )
    ])
```

## 4. Processing Sensor Data

Now, let's create a ROS 2 Python node that subscribes to the `lidar_scan` topic and processes the incoming `LaserScan` messages.

**File: `code-examples/module2-the-digital-twin/chapter2.3/scripts/sensor_processor.py`**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar_scan',
            self.lidar_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('LiDAR Processor node started, subscribing to /lidar_scan')

    def lidar_callback(self, msg: LaserScan):
        """
        Callback function for processing incoming LaserScan messages.
        Finds the closest obstacle in the scan.
        """
        closest_range = float('inf')
        closest_angle = 0.0

        for i, range_val in enumerate(msg.ranges):
            if msg.range_min <= range_val <= msg.range_max:
                if range_val < closest_range:
                    closest_range = range_val
                    angle = msg.angle_min + i * msg.angle_increment
                    closest_angle = math.degrees(angle)

        if closest_range != float('inf'):
            self.get_logger().info(f'Closest obstacle: {closest_range:.2f} m at {closest_angle:.2f} degrees')
        else:
            self.get_logger().info('No obstacles detected within range.')

def main(args=None):
    rclpy.init(args=args)
    try:
        lidar_processor = LidarProcessor()
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

By completing this chapter, you've equipped your digital twin with basic perception capabilities, allowing it to sense and understand its virtual world. This is a crucial step towards developing autonomous behaviors.