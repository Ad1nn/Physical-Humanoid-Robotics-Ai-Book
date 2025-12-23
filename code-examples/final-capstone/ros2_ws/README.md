# ROS 2 Workspace for Capstone Project

This directory (`ros2_ws`) is a placeholder for your ROS 2 workspace for the Final Capstone Project.

## Project Structure

Within this `ros2_ws`, you will create several ROS 2 packages under the `src` subdirectory, each handling a specific part of the VLA system.

```
ros2_ws/
├── src/
│   ├── voice_command_node/ # Handles Whisper integration for STT
│   │   ├── package.xml
│   │   └── voice_command_node.py
│   ├── llm_planner_node/ # Interfaces with LLM for cognitive planning
│   │   ├── package.xml
│   │   └── llm_planner_node.py
│   ├── vla_control_node/ # Orchestrates overall VLA system, integrates perception, Nav2, manipulation
│   │   ├── package.xml
│   │   └── vla_control_node.py
│   └── custom_perception_node/ # Example custom node for object detection (optional)
│       └── ...
├── install/
├── log/
└── build/
```

## Setup Instructions

1.  **Create your ROS 2 packages**: For each required component (voice command, LLM planner, VLA control), create a new Python ROS 2 package:
    ```bash
    cd ros2_ws/src
    ros2 pkg create --build-type ament_python voice_command_node --dependencies rclpy std_msgs sensor_msgs
    ros2 pkg create --build-type ament_python llm_planner_node --dependencies rclpy std_msgs
    ros2 pkg create --build-type ament_python vla_control_node --dependencies rclpy std_msgs geometry_msgs nav_msgs
    ```
2.  **Copy your node scripts**: Place your Python scripts (`voice_command_node.py`, `llm_planner_node.py`, `vla_control_node.py`) into the appropriate package subdirectories.
3.  **Update `setup.py`**: Add entry points for your nodes in each package's `setup.py`.
4.  **Build the workspace**:
    ```bash
    cd ros2_ws
    colcon build
    ```
5.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Running the Project

The integrated system will be launched using a single launch file:

```bash
ros2 launch vla_capstone_launch capstone_launch.launch.py # Assuming vla_capstone_launch is a package
```

This README provides the basic guidance. Refer to the capstone project guide (`docs/module-04-vla/project.mdx`) for full details.
