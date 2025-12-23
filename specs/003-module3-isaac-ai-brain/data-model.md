# Data Models: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

This document describes the key data entities and structures used in the "AI-Robot Brain" module, focusing on NVIDIA Isaac Sim, Isaac ROS, and Nav2.

## 1. USD (Universal Scene Description)

-   **Entity**: Isaac Sim Scene / Assets
-   **Format**: Universal Scene Description (.usd, .usdc, .usda)
-   **Description**: USD is NVIDIA Isaac Sim's primary format for scene description, asset interchange, and collaboration. It's a powerful framework for composing, assembling, and editing 3D scenes.
-   **Key Attributes/Concepts**:
    -   **Prims**: The fundamental building blocks of a USD scene (e.g., shapes, lights, cameras, materials).
    -   **Layers**: Non-destructive editing mechanism, allowing multiple users or tools to contribute to a single scene.
    -   **References/Payloads**: Mechanism to compose larger scenes from smaller, reusable assets.
    -   **Xforms**: Transformations (translate, rotate, scale) applied to prims.
    -   **Relationships**: Connections between prims (e.g., parent-child, material binding).

## 2. Isaac ROS Message Types

-   **Entity**: Sensor Data, VSLAM Output
-   **Format**: ROS 2 Messages
-   **Description**: Standardized ROS 2 message types, often with specific optimizations or extensions for hardware acceleration provided by Isaac ROS.
-   **Key Message Types**:
    -   `sensor_msgs/msg/Image`: Raw or processed image data from cameras (stereo, RGB-D).
    -   `sensor_msgs/msg/CameraInfo`: Camera intrinsic and extrinsic parameters.
    -   `sensor_msgs/msg/PointCloud2`: 3D point cloud data from LiDAR or depth cameras.
    -   `geometry_msgs/msg/PoseStamped`: A pose (position and orientation) with a timestamp, used for VSLAM output.
    -   `nav_msgs/msg/Odometry`: Odometry information (position, velocity) typically fused from multiple sources.
    -   `isaac_ros_visual_slam_interfaces/msg/VisualSlamStats`: Custom message for VSLAM pipeline statistics.

## 3. Nav2 Configuration and Runtime Data

-   **Entity**: Navigation Map, Robot State, Planner Output
-   **Format**: YAML configuration files, ROS 2 Messages
-   **Description**: Nav2 uses various configuration files and ROS 2 messages to define the environment, robot characteristics, planning algorithms, and execution.
-   **Key Data/Configuration Types**:
    -   **Costmaps**: Grid-based representations of the environment, storing information about obstacles and inflation layers. Defined via YAML configuration.
        -   `global_costmap.yaml`: Configuration for the global costmap (used by global planner).
        -   `local_costmap.yaml`: Configuration for the local costmap (used by local planner).
    -   **Planner/Controller Parameters**: YAML files defining parameters for path planning algorithms (e.g., DWA, TEB) and controllers.
    -   `nav_msgs/msg/OccupancyGrid`: A common ROS 2 message for representing grid maps.
    -   `geometry_msgs/msg/PoseStamped`: Goal poses for navigation.
    -   `geometry_msgs/msg/Twist`: Velocity commands sent to the robot base.

## 4. Synthetic Data

-   **Entity**: Generated Perception Data
-   **Format**: Image files (PNG, JPG), Point Cloud files (PLY, PCAP), ROS bag files.
-   **Description**: Data generated within Isaac Sim to simulate real-world sensor outputs for training AI models.
-   **Key Attributes**:
    -   **RGB Images**: Standard color images.
    -   **Depth Images**: Grayscale images where pixel intensity represents distance.
    -   **Semantic Segmentation Masks**: Images where each pixel identifies the class of the object it belongs to.
    -   **LiDAR Point Clouds**: Collections of 3D points representing the environment.
    -   **Ground Truth Poses**: The exact, true position and orientation of the robot/sensors in the simulation.
