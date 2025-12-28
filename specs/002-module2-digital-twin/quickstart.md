# Quickstart: Module 2 Environment Setup

This guide provides the essential steps to configure your environment for the Module 2 "Digital Twin" hands-on examples.

## Prerequisites

1.  **Ubuntu 22.04 LTS**: Or a Docker container running it.
2.  **ROS 2 Humble**: Fully installed and sourced.
3.  **Module 1 Completion**: You should be familiar with creating ROS 2 nodes and packages.
4.  **Hardware**: 16GB RAM and an NVIDIA GPU (6GB+ VRAM) are strongly recommended.

## 1. Install Gazebo Garden

Gazebo Garden is the latest version of the Gazebo simulator.

```bash
# Add the Gazebo repository to your sources list
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt-get update
sudo apt-get install gz-garden
```

## 2. Install ROS 2-Gazebo Bridge

This package provides the necessary plugins to connect ROS 2 with Gazebo.

```bash
sudo apt-get install ros-humble-ros-gz
```

This installs `ros_gz_sim`, `ros_gz_bridge`, and `ros_gz_interfaces`.

## 3. Install Unity Hub and Unity Editor

1.  Download and install the **Unity Hub**.
2.  From the Unity Hub, install **Unity Editor version 2022.3.x LTS**. Ensure you include Linux build support if prompted.

## 4. Configure Unity for ROS 2

1.  Create a new 3D project in the Unity Hub.
2.  Go to `Window -> Package Manager`.
3.  Click the `+` icon and select `Add package from git URL...`.
4.  Enter `com.unity.robotics.ros-tcp-connector` and click `Add`. This will install the ROS-TCP-Connector package.
5.  In your Unity project, go to `Robotics -> ROS Settings` to configure the connection to your ROS 2 environment (by default, it will attempt to connect to localhost).

## 5. Clone the Book's Code Repository

If you haven't already, clone the repository containing the code examples.

```bash
git clone <repository_url>
cd physical-ai-book/code-examples/module2-the-digital-twin
```

You are now ready to run the launch files and simulation examples provided in this module.
