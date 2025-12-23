# Data Models: Module 2

This document describes the key data entities and structures used in the "Digital Twin" module. These are not database schemas but definitions of the file formats and message contracts that form the core of the simulation.

## 1. Humanoid Robot Model

-   **Entity**: Humanoid Robot
-   **Format**: URDF (Unified Robot Description Format) / XACRO
-   **Description**: An XML-based format used by ROS to describe all physical aspects of a robot. This includes the robot's links (physical parts), joints (connections between links), visual meshes, collision properties, and inertial data. XACRO is a macro language that is used to make URDF files more readable and modular.
-   **Key Attributes**:
    -   `<link name="...">`: Defines a rigid body part of the robot. Contains `<visual>`, `<collision>`, and `<inertial>` child elements.
    -   `<joint name="..." type="...">`: Defines the kinematics and dynamics of a connection between two links. Common types include `revolute`, `continuous`, `prismatic`, and `fixed`.
    -   `<gazebo reference="...">`: A tag used to specify Gazebo-specific properties, such as sensor plugins, materials, and physics parameters.

## 2. Simulation World

-   **Entity**: Gazebo World
-   **Format**: SDF (Simulation Description Format)
-   **Description**: An XML-based format used by Gazebo to describe the entire simulation environment, including robots, lights, physics properties, and static objects.
-   **Key Attributes**:
    -   `<world name="...">`: The root element of the file.
    -   `<physics type="...">`: Defines the physics engine (e.g., `ode`, `bullet`) and global parameters like gravity.
    -   `<include><uri>...</uri></include>`: Used to load models (like robots or obstacles) into the world from their own files.
    -   `<light type="...">`: Defines light sources in the environment.
    -   `<scene><grid>...</grid></scene>`: Configures visual aspects of the scene, such as the ground plane grid.

## 3. Joint Control Messages

-   **Entity**: Joint State & Command
-   **Format**: ROS 2 Messages
-   **Description**: Standardized message types used for reporting and commanding the state of robot joints.
-   **Key Message Types**:
    -   `sensor_msgs/msg/JointState`: Used to publish the current state (position, velocity, effort) of multiple joints.
        -   `string[] name`
        -   `float64[] position`
        -   `float64[] velocity`
        -   `float64[] effort`
    -   `trajectory_msgs/msg/JointTrajectory`: Used to send a sequence of points for a robot to follow.
        -   `string[] joint_names`
        -   `trajectory_msgs/msg/JointTrajectoryPoint[] points`

## 4. Sensor Data Messages

-   **Entity**: Sensor Data
-   **Format**: ROS 2 Messages
-   **Description**: Standardized message types for publishing data from simulated sensors.
-   **Key Message Types**:
    -   `sensor_msgs/msg/LaserScan`: For 2D LiDAR data. Contains an array of range readings.
    -   `sensor_msgs/msg/PointCloud2`: For 3D LiDAR and depth camera data. A flexible format for representing N-dimensional point clouds.
    -   `sensor_msgs/msg/Image`: For RGB camera data.
    -   `sensor_msgs/msg/Imu`: For Inertial Measurement Unit data, including orientation, angular velocity, and linear acceleration.
