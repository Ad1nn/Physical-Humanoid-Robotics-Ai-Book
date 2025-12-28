# Quickstart: Module 3 Environment Setup - NVIDIA Isaac Sim & Isaac ROS

This guide provides the essential steps to configure your environment for the Module 3 "AI-Robot Brain" hands-on examples. NVIDIA Isaac Sim and Isaac ROS require specific hardware and software configurations.

## Prerequisites

1.  **Ubuntu 22.04 LTS**: Or a Docker container running it.
2.  **NVIDIA GPU**: Minimum 8GB+ VRAM (e.g., RTX 3060, RTX 2070, or better).
3.  **NVIDIA Drivers**: Latest proprietary NVIDIA drivers installed.
4.  **Docker & NVIDIA Container Toolkit**: Properly installed and configured for GPU access.
5.  **ROS 2 Humble**: Fully installed and sourced in your base environment or within your development container.
6.  **Module 1 & 2 Completion**: Understanding of ROS 2 basics and general simulation concepts.

## 1. Install NVIDIA Isaac Sim

Isaac Sim is distributed as a Docker image.

1.  **Pull the Isaac Sim Docker Image**:
    ```bash
    docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
    ```
    (Ensure you are pulling the version specified in the module or a compatible later version).

2.  **Launch Isaac Sim**:
    ```bash
    ./run_isaac_sim.sh
    ```
    *(Note: You will typically find a `run_isaac_sim.sh` script in your Isaac Sim installation directory after downloading from NVIDIA Developer Zone. This script handles the complex Docker command with GPU passthrough, X-server access, etc. Students should download Isaac Sim from the NVIDIA Developer Zone for the full installation experience.)*

3.  **Verify Launch**: Once launched, you should see the Isaac Sim UI.

## 2. Setup Isaac ROS Workspace

Isaac ROS packages are also often used within a ROS 2 workspace, frequently integrated within Isaac Sim's container or a separate Docker container with shared volumes.

1.  **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/isaac_ros_ws/src
    cd ~/isaac_ros_ws/src
    ```

2.  **Clone Isaac ROS Repositories**:
    (Specific repositories will depend on the Isaac ROS version and features. Example for VSLAM:)
    ```bash
    git clone https://github.com/NVIDIA-AI-ROBOTICS/isaac_ros_common.git
    git clone https://github.com/NVIDIA-AI-ROBOTICS/isaac_ros_visual_slam.git
    # ... other Isaac ROS packages as needed
    ```

3.  **Install Dependencies and Build**:
    *(This step is typically done inside a Docker container provided by Isaac ROS or derived from Isaac Sim.)*
    ```bash
    cd ~/isaac_ros_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --merge-install
    ```

4.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```

## 3. Configure Nav2 Stack

Nav2 configuration involves setting up various YAML files for costmaps, planners, and controllers. These will be placed in your ROS 2 package.

*(Note: Detailed Nav2 setup will be covered in Chapter 3.3. This quickstart provides a high-level overview.)*

## 4. Clone the Book's Code Repository

If you haven't already, clone the repository containing the code examples.

```bash
git clone <repository_url>
cd physical-ai-book/code-examples/module3-isaac-ai-brain
```

You are now ready to run the Isaac Sim scenes and Isaac ROS examples provided in this module.
