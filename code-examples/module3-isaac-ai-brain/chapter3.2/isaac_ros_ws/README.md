# Isaac ROS Workspace Setup Guide

This directory contains resources and instructions for setting up your Isaac ROS workspace for Chapter 3.2.

## Important Prerequisites

-   **ROS 2 Humble**: Fully installed and sourced.
-   **Docker & NVIDIA Container Toolkit**: Properly installed and configured for GPU access.
-   **Isaac Sim**: Installed and functional (from Chapter 3.1 setup). While not always directly needed for Isaac ROS, it's the primary simulation environment.

## Step 1: Create a ROS 2 Workspace

We will create a standard ROS 2 workspace to host our Isaac ROS packages.

```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
```

## Step 2: Clone Isaac ROS Repositories

Isaac ROS is a collection of various packages. For VSLAM and perception, you'll need the following:

```bash
# Clone common Isaac ROS utilities
git clone https://github.com/NVIDIA-AI-ROBOTICS/isaac_ros_common.git

# Clone visual SLAM package
git clone https://github.com/NVIDIA-AI-ROBOTICS/isaac_ros_visual_slam.git

# Clone stereo_image_proc (for stereo depth processing, if used)
git clone https://github.com/ros-perception/image_pipeline.git -b humble # Use the humble branch
```

## Step 3: Install Dependencies and Build

This step is typically performed within a Docker container (e.g., one derived from the Isaac Sim container or a dedicated Isaac ROS container provided by NVIDIA) to ensure all dependencies are met.

```bash
# Navigate to your workspace root
cd ~/isaac_ros_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --merge-install
```

## Step 4: Source the Workspace

After building, source your workspace to make the Isaac ROS packages available in your environment.

```bash
source install/setup.bash
```

## Step 5: Verify Isaac ROS Packages

You can verify the installation by listing available ROS 2 packages:

```bash
ros2 pkg list | grep isaac_ros
```
You should see `isaac_ros_common`, `isaac_ros_visual_slam`, etc. listed.

For detailed setup and troubleshooting, refer to the [official Isaac ROS documentation](https://developer.nvidia.com/isaac-ros/documentation).
