---
sidebar_position: 6
title: 'Module 3: Assessment'
---

# Module 3: Assessment

This assessment evaluates your understanding of advanced robot perception and navigation using the NVIDIA Isaac platform, including Isaac Sim, Isaac ROS, and Nav2.

## Section 1: Multiple Choice Questions (10 Questions)

1.  What is the primary file format used by NVIDIA Isaac Sim for describing 3D scenes and assets?
    a) URDF
    b) SDF
    c) USD
    d) OBJ

2.  Which of the following is a key advantage of synthetic data generation in Isaac Sim for training AI models?
    a) Eliminates the need for GPU hardware.
    b) Provides perfect ground truth data and avoids manual labeling.
    c) Only works with real-world sensor data.
    d) Is primarily used for physics-only simulations.

3.  What does VSLAM stand for?
    a) Visual Sensor Localization and Mapping
    b) Velocity-based Simultaneous Localization and Mapping
    c) Visual Simultaneous Localization and Mapping
    d) Virtual Simulation and Live Mapping

4.  Which NVIDIA platform provides hardware-accelerated ROS 2 packages for perception and navigation?
    a) Isaac Gym
    b) Jetson Nano
    c) Isaac ROS
    d) DeepStream

5.  What is a major challenge when adapting the Nav2 stack for bipedal (humanoid) robots compared to wheeled robots?
    a) Lack of suitable global planners.
    b) Simpler kinematics and dynamics.
    c) Complex dynamic balance and footstep planning requirements.
    d) Inability to use costmaps.

6.  In Nav2, what is the purpose of an "inflation layer" in a costmap?
    a) To increase the robot's physical size.
    b) To assign a higher cost to areas near obstacles, providing clearance.
    c) To deflate obstacles for easier navigation.
    d) To represent the robot's battery level.

7.  Which Isaac ROS package is specifically designed for optimized Visual SLAM solutions?
    a) `isaac_ros_image_pipeline`
    b) `isaac_ros_nvgstreamer`
    c) `isaac_ros_visual_slam`
    d) `isaac_ros_argus_camera`

8.  What is a common strategy to mitigate VSLAM drift in challenging environments?
    a) Only use static environments.
    b) Integrate additional sensors like IMUs.
    c) Reduce the camera's frame rate.
    d) Increase the robot's speed.

9.  Which Nav2 component is responsible for executing a global path while avoiding dynamic obstacles in the immediate vicinity?
    a) Global Planner
    b) Behavior Tree Navigator
    c) Local Controller/Planner
    d) Map Server

10. What is a key hardware requirement for optimal performance with NVIDIA Isaac Sim and Isaac ROS?
    a) High-speed CPU with integrated graphics.
    b) Any GPU with at least 4GB VRAM.
    c) NVIDIA GPU with minimum 8GB+ VRAM.
    d) ARM-based processor for low power consumption.

## Section 2: Practical Coding Challenges

You must complete at least 2 out of the 3 challenges.

### Challenge 1: Tune Isaac Sim Scene for Performance

**Goal**: Optimize an Isaac Sim scene to meet a specific performance target.

-   **Description**: You will be provided with an `unoptimized_scene.usd` file in Isaac Sim that runs below 30 FPS.
-   **Task**: Identify and implement scene optimizations (e.g., reduce mesh complexity, adjust physics steps, optimize lighting, disable unnecessary features) to achieve a stable simulation frame rate of at least 30 FPS.
-   **Deliverable**: Submit the modified `optimized_scene.usd` file and a brief report detailing the optimizations made and the resulting FPS.

### Challenge 2: Visualize VSLAM Output in RViz2

**Goal**: Extend the VSLAM pipeline to clearly visualize its output in RViz2.

-   **Description**: Using the `vslam_pipeline.launch.py` from Chapter 3.2, ensure all relevant VSLAM output topics are published.
-   **Task**: Create a custom RViz2 configuration file (`vslam_display.rviz`) that visualizes:
    -   The robot's estimated pose (`/visual_slam/tracking/slam_pose`).
    -   The generated map (e.g., point cloud map, if available from VSLAM).
    -   The camera frames used by VSLAM.
-   **Deliverable**: Submit the `vslam_display.rviz` configuration file and instructions on how to launch it alongside your VSLAM pipeline.

### Challenge 3: Custom Nav2 Behavior Tree

**Goal**: Modify Nav2's behavior tree to implement a custom recovery behavior.

-   **Description**: Nav2 uses Behavior Trees (BTs) to define navigation logic. You will be provided with a standard `navigate_to_pose.xml` BT file.
-   **Task**: Modify the provided BT to add a new recovery behavior: if the robot fails to reach its goal after 3 attempts, it should "spin in place" once for 5 seconds before trying again.
-   **Deliverable**: Submit the modified `.xml` behavior tree file.

## Assessment Rubric

### Multiple Choice (50% of total grade)

-   Each question is worth 5 points.
-   Correct answers demonstrate theoretical understanding of Isaac platform, VSLAM, and Nav2 concepts.

### Practical Challenges (50% of total grade)

-   Each challenge is worth 25 points (if you choose 2 challenges, each is 25; if you choose all 3, they are each ~16.6 points for a total of 50).
-   **Functionality (15 points)**: Solution runs without errors and meets all technical requirements.
-   **Code/Config Quality (5 points)**: Files are well-structured, commented, and adhere to best practices.
-   **Correctness (5 points)**: The solution accurately addresses the problem statement (e.g., optimizations achieve FPS target, RViz2 displays correctly, BT implements new behavior).

**Passing Criteria**: A combined score of 75% or higher is required to pass Module 3.
