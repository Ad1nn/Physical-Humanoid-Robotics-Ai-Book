# Research & Decisions: Module 2

This document records the key technical research and decisions made during the planning phase for Module 2.

## 1. Gazebo-ROS 2 Integration

-   **Topic**: Best practices for integrating Gazebo Garden/Harmonic with ROS 2 Humble.
-   **Decision**: Utilize the `gz_ros2_control` plugin.
-   **Rationale**: This is the standard and most robust method for controlling robot joints in Gazebo from a ROS 2 environment. It provides a seamless bridge between ROS 2 control messages and Gazebo's physics-based joint controllers, simplifying what would otherwise be a complex integration. It is well-documented and supported by the community.
-   **Alternatives Considered**:
    -   **Custom Gazebo Plugins**: Writing a custom C++ plugin to interface with ROS 2. This offers maximum flexibility but is overly complex for the educational goals of this module and would distract from the core learning objectives.
    -   **Direct Gazebo Transport**: Communicating directly with Gazebo's transport layer from ROS 2. This is less standardized and requires more boilerplate code than using `gz_ros2_control`.

## 2. Unity-ROS 2 Integration

-   **Topic**: Optimal method for connecting a Unity scene with a ROS 2 network.
-   **Decision**: Use the official Unity Robotics Hub `ROS-TCP-Connector` package.
-   **Rationale**: This is the Unity-sanctioned approach for ROS integration. It is actively maintained, well-documented, and handles the complexities of serialization and transport over a TCP connection. It provides a straightforward component-based workflow within the Unity editor, which is ideal for students.
-   **Alternatives Considered**:
    -   **WebSocket-based Bridge**: Implementing a custom WebSocket server in ROS 2 and a client in Unity. This introduces unnecessary complexity and potential performance overhead compared to the purpose-built TCP connector.
    -   **Writing a custom C# UDP solution**: This would be a significant engineering effort and is not suitable for the scope of this educational module.

## 3. Physics Engine Selection

-   **Topic**: Choice of physics engines to highlight within Gazebo.
-   **Decision**: The content will primarily use the default (Trivial Physics Engine in recent Gazebo versions, or DART/Bullet as common options) but will include a brief comparison table mentioning ODE, Bullet, and DART.
-   **Rationale**: For the learning objectives, the specific choice of physics engine is less important than understanding the concepts of physics simulation itself. Focusing on the default engine simplifies the setup for students. Providing a comparison table gives them awareness of the options and their trade-offs (e.g., performance vs. accuracy) for future, more advanced work.
-   **Alternatives Considered**: Mandating a specific non-default engine (e.g., DART) for all examples. This would add an extra configuration step for students without providing a significant corresponding educational benefit for this module's scope.
