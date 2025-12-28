---
sidebar_position: 5
title: 'Module 2: Hands-on Project'
---

# Module 2: Hands-on Project - Custom Humanoid Simulation

## Project Goal

In this hands-on project, you will apply the knowledge gained in Module 2 to create a custom simulation environment for a humanoid robot. You will design a unique Gazebo world, integrate a humanoid model with sensors, and establish a ROS 2 bridge to control and perceive its state.

## Scenario

Imagine you are tasked with developing a humanoid robot for assistive tasks in a cluttered home environment. Before deploying the physical robot, you need to set up a robust simulation to test its navigation and interaction capabilities.

## Deliverables

1.  **Custom Gazebo World**: Create an SDF file for a Gazebo world that simulates a small home environment. Include:
    -   A ground plane.
    -   At least 3 different static obstacles (e.g., table, chair, wall).
    -   Proper lighting.
    -   Physics properties (e.g., friction for the floor).
    Save this as `code-examples/module2-the-digital-twin/project/custom_home.sdf`.

2.  **Sensor-Equipped Humanoid**: Take the `humanoid_base.urdf.xacro` model and enhance it by:
    -   Adding at least two sensors: one LiDAR and one Depth Camera.
    -   Ensuring the sensors are correctly positioned and configured to publish data on ROS 2 topics.
    -   Save this as `code-examples/module2-the-digital-twin/project/humanoid_project.urdf.xacro`.

3.  **Integrated Launch File**: Create a ROS 2 launch file that:
    -   Starts Gazebo with your `custom_home.sdf` world.
    -   Spawns your `humanoid_project.urdf.xacro` model into the world.
    -   Starts `robot_state_publisher` and `ros_gz_bridge` (if necessary for your sensor topics).
    -   Save this as `code-examples/module2-the-digital-twin/project/launch/project_launch.launch.py`.

4.  **Sensor Data Processing Node**: Write a Python ROS 2 node that:
    -   Subscribes to the data from *both* the LiDAR and Depth Camera sensors.
    -   Processes the data (e.g., finds the closest object from LiDAR, identifies average depth in a region from the depth camera).
    -   Logs the processed information to the console.
    -   Save this as `code-examples/module2-the-digital-twin/project/scripts/data_fusion_node.py`.

## Submission

-   All code and configuration files should be placed in a new directory: `code-examples/module2-the-digital-twin/project/`.
-   A brief `README.md` file in this directory explaining how to run your simulation and the expected output from your sensor processing node.

Good luck, and have fun building your digital twin!
