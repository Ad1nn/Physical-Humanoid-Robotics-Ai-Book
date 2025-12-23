# ROS 2 Workspace for Autonomous Navigation Project

This directory (`ros2_ws`) is a placeholder for your ROS 2 workspace for the Autonomous Humanoid Navigation Project.

## Project Structure

Within this `ros2_ws`, you will typically create your ROS 2 packages under the `src` subdirectory.

```
ros2_ws/
├── src/
│   ├── autonomous_humanoid_nav/ # Your custom ROS 2 package
│   │   ├── launch/
│   │   │   └── autonomous_navigation.launch.py
│   │   ├── scripts/
│   │   │   └── goal_publisher.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── README.md
├── install/
├── log/
└── build/
```

## Setup Instructions

1.  **Create your ROS 2 package**:
    ```bash
    cd ros2_ws/src
    ros2 pkg create --build-type ament_python autonomous_humanoid_nav --dependencies rclpy nav2_msgs geometry_msgs
    ```
2.  **Copy your launch and script files**: Place your `autonomous_navigation.launch.py` and `goal_publisher.py` (once created) into the appropriate directories within your new package.
3.  **Update `setup.py`**: Add entry points for your scripts in your package's `setup.py`.
4.  **Build the workspace**:
    ```bash
    cd ros2_ws
    colcon build --packages-select autonomous_humanoid_nav
    ```
5.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Running the Project

Once set up, you will typically launch your project using:

```bash
ros2 launch autonomous_humanoid_nav autonomous_navigation.launch.py
```

This README provides the basic guidance. Refer to the project guide (`docs/module-03-isaac/project.mdx`) for full details.
