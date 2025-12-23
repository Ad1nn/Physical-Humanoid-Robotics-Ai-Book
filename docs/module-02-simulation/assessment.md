---
sidebar_position: 6
title: 'Module 2: Assessment'
---

# Module 2: Assessment

This assessment evaluates your understanding of robotics simulation using Gazebo and Unity, as well as your ability to integrate these platforms with ROS 2.

## Section 1: Multiple Choice Questions (10 Questions)

1.  Which file format is primarily used by Gazebo to describe an entire simulation environment, including physics properties, lights, and models?
    a) URDF
    b) XACRO
    c) SDF
    d) YAML

2.  What is the main advantage of using Unity over Gazebo for robotics simulation, especially for human-robot interaction?
    a) Better physics accuracy
    b) More streamlined ROS 2 integration
    c) Higher fidelity rendering and rich interactive environments
    d) Lower computational requirements

3.  Which ROS 2 package is commonly used to spawn URDF models into Gazebo?
    a) `robot_state_publisher`
    b) `ros_gz_sim`
    c) `gazebo_ros2_control`
    d) `ament_index_python`

4.  The `ROS-TCP-Connector` in Unity facilitates communication with ROS 2 over what protocol?
    a) UDP
    b) TCP
    c) HTTP
    d) WebSocket

5.  What Unity component is specifically designed for simulating the joints of robotic arms and kinematic chains, offering more stability than standard rigidbodies?
    a) Rigidbody
    b) Joint
    c) ArticulationBody
    d) HingeJoint

6.  Which ROS 2 message type is typically used for 2D LiDAR data?
    a) `sensor_msgs/msg/Image`
    b) `sensor_msgs/msg/PointCloud2`
    c) `sensor_msgs/msg/JointState`
    d) `sensor_msgs/msg/LaserScan`

7.  What does an IMU (Inertial Measurement Unit) primarily measure?
    a) Distance to obstacles
    b) Color and depth information
    c) Orientation, angular velocity, and linear acceleration
    d) Force and torque applied to joints

8.  In an URDF, what element is used to define Gazebo-specific properties for a link, such as sensor plugins?
    a) `<link>`
    b) `<joint>`
    c) `<gazebo>`
    d) `<material>`

9.  Which of the following is NOT a primary reason for using simulation in robotics development?
    a) Cost reduction
    b) Safety
    c) Real-world testing in uncontrolled environments
    d) Rapid prototyping

10. If you are developing a ROS 2 application that needs to control a robot's joints, which ROS 2 message type is most appropriate for sending position commands over a period of time?
    a) `std_msgs/msg/Float64`
    b) `sensor_msgs/msg/JointState`
    c) `trajectory_msgs/msg/JointTrajectory`
    d) `geometry_msgs/msg/Twist`

## Section 2: Practical Coding Challenges

You must complete at least 2 out of the 3 challenges.

### Challenge 1: Stabilize Humanoid Physics

**Goal**: Modify a given Gazebo world or URDF to stabilize a humanoid robot that is exhibiting unstable physics behavior (e.g., falling over immediately).

-   **Description**: You will be provided with a `humanoid_unstable.urdf.xacro` and `unstable_world.sdf` files.
-   **Task**: Identify and adjust the physics parameters (e.g., `<inertial>` values, joint limits, friction, damping in `unstable_world.sdf` or the URDF) to make the humanoid stand stably.
-   **Deliverable**: Submit the modified `humanoid_unstable.urdf.xacro` and `unstable_world.sdf` files.

### Challenge 2: Integrate a New Sensor

**Goal**: Add a new type of sensor to a humanoid robot and demonstrate its data output.

-   **Description**: Start with your `humanoid_base.urdf.xacro` (or the `humanoid_ch2.1.urdf.xacro` from Chapter 2.1).
-   **Task**: Add a simulated **Depth Camera** to the humanoid's head.
    -   Configure the camera to publish `sensor_msgs/msg/Image` (for RGB) and `sensor_msgs/msg/PointCloud2` (for depth data).
    -   Create a ROS 2 launch file to spawn the humanoid with the depth camera in Gazebo.
    -   Create a simple Python ROS 2 node to subscribe to the depth data (PointCloud2) and print the average depth of the scene.
-   **Deliverable**: Submit the modified URDF, launch file, and Python subscriber node.

### Challenge 3: Custom Gazebo Environment

**Goal**: Create a unique Gazebo world with specific constraints for a robot to navigate.

-   **Description**: Design a new Gazebo world.
-   **Task**: Create an `apartment.sdf` world file that includes:
    -   A floor and at least two walls forming a corner.
    -   A ramp or staircase.
    -   One light source.
    -   Spawn a simple `box` model (Gazebo's built-in primitive) into this world.
-   **Deliverable**: Submit the `apartment.sdf` file and a launch file to open it in Gazebo.

## Assessment Rubric

### Multiple Choice (50% of total grade)

-   Each question is worth 5 points.
-   Correct answers demonstrate theoretical understanding of simulation concepts.

### Practical Challenges (50% of total grade)

-   Each challenge is worth 25 points (if you choose 2 challenges, each is 25; if you choose all 3, they are each ~16.6 points for a total of 50).
-   **Functionality (15 points)**: Code runs without errors, produces expected output, and meets the challenge requirements.
-   **Code Quality (5 points)**: Code is readable, well-commented, and adheres to basic Python/ROS 2 best practices.
-   **Correctness (5 points)**: The solution accurately addresses the problem statement (e.g., robot is stable, sensor data is processed correctly, world includes all elements).

**Passing Criteria**: A combined score of 80% or higher is required to pass Module 2.
