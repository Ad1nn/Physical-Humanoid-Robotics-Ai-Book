# Dockerfile for ROS 2 Humble Development Environment
# This image provides a consistent environment for all Module 1 code examples.

# Use official Ubuntu 22.04 as base
FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Update package list and install necessary tools
RUN apt update && apt install -y \
    locales \
    software-properties-common \
    curl \
    git \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install ROS 2 Humble
# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Add ROS 2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update again and install ROS 2 Humble desktop environment
RUN apt update && apt install -y \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup script automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO humble

# Create a workspace for ROS 2 development
WORKDIR /ros_ws
RUN mkdir -p src

# Copy local code examples and install dependencies
# This assumes the code examples will be mounted or copied here
COPY requirements.txt /ros_ws/
RUN pip install --no-cache-dir -r /ros_ws/requirements.txt

# Set default command to bash
CMD ["bash"]
