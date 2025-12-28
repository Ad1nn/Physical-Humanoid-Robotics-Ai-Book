---
sidebar_position: 5
title: 'Module 3: Hands-on Project'
---

# Module 3: Hands-on Project - Autonomous Humanoid Navigation

## Project Goal

In this hands-on project, you will integrate the concepts learned throughout Module 3 to enable a humanoid robot to autonomously navigate an indoor environment within NVIDIA Isaac Sim. This involves setting up the Isaac Sim scene, utilizing Isaac ROS VSLAM for localization, and configuring the Nav2 stack for bipedal path planning and dynamic obstacle avoidance.

## Scenario

Your humanoid robot is tasked with delivering a package across a simulated apartment complex. It needs to navigate from a starting point to a series of waypoints, avoiding static and dynamic obstacles, and reporting its progress.

## Deliverables

1.  **Isaac Sim Scene**: Create a detailed Isaac Sim scene (USD file) representing an indoor apartment environment. This scene should include:
    -   Walls, doors, furniture (static obstacles).
    -   A moving obstacle (e.g., another robot or person) to simulate dynamic avoidance.
    -   The humanoid robot model equipped with stereo cameras (for VSLAM) and a LiDAR sensor (for Nav2 costmaps).
    Save this as `code-examples/module3-isaac-ai-brain/project/isaac_sim_scene/autonomous_env.usd`.

2.  **Integrated ROS 2 Workspace**: Set up a ROS 2 workspace for the project (`code-examples/module3-isaac-ai-brain/project/ros2_ws/`). This workspace will contain:
    -   A launch file to bring up Isaac Sim with your custom scene and humanoid.
    -   Launch Isaac ROS VSLAM pipeline for localization and mapping.
    -   Launch the Nav2 stack configured for your humanoid.
    -   Save the main launch file as `code-examples/module3-isaac-ai-brain/project/ros2_ws/launch/autonomous_navigation.launch.py`.

3.  **Goal Publishing Script**: Write a Python ROS 2 script that:
    -   Connects to the Nav2 stack.
    -   Publishes a series of `geometry_msgs/msg/PoseStamped` goals to guide the humanoid through the environment.
    -   Monitors the robot's progress towards each goal.
    -   Save this as `code-examples/module3-isaac-ai-brain/project/scripts/goal_publisher.py`.

## Testing Scenarios

-   **Hallway Navigation**: Robot navigates a straight hallway without collision.
-   **Doorway Passing**: Robot successfully passes through an open doorway.
-   **Dynamic Obstacle Avoidance**: Robot avoids a moving obstacle in its path and reaches the goal.
-   **Crowded Spaces**: Robot navigates an area with multiple static obstacles.

## Expected Outcomes

-   Humanoid robot autonomously navigates to multiple waypoints.
-   Robot avoids static and dynamic obstacles gracefully.
-   Accurate localization and mapping via VSLAM.
-   Nav2 stack successfully plans and executes paths for bipedal locomotion.

## Submission

-   All code and configuration files should be placed in `code-examples/module3-isaac-ai-brain/project/`.
-   A brief `README.md` file in this directory explaining how to run your autonomous navigation project.

Good luck, and enjoy building your AI-powered humanoid!
