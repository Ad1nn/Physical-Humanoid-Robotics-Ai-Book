---
sidebar_position: 3
title: '2.2: Unity High-Fidelity Rendering'
---

# Chapter 2.2: Unity High-Fidelity Rendering

While Gazebo is a powerful tool for physics simulation, the **Unity** game engine allows us to create visually stunning and highly interactive environments. In this chapter, you'll learn why Unity is becoming a go-to tool for robotics, how to import your robot, and how to control it from ROS 2.

## 1. Why Unity for Robotics?

-   **Photorealistic Rendering**: Unity's High Definition Render Pipeline (HDRP) enables lifelike visuals, which is critical for training vision-based AI.
-   **Rich Asset Store**: Access to a massive ecosystem of 3D models, textures, and tools.
-   **Human-in-the-Loop**: Game engine features make it easy to create scenarios where a human can interact with the robot, for example, by controlling an avatar in the same scene.
-   **C# Scripting**: A powerful, modern, and widely-used language for implementing complex logic within the simulation.

## 2. Setting up the Unity-ROS 2 Bridge

Unity communicates with ROS 2 via the `ROS-TCP-Connector` package. This package creates a TCP connection between your Unity application and a ROS 2 node, relaying messages back and forth.

**Setup Steps**:

1.  Create a new 3D project in Unity Hub (2022.3.x LTS).
2.  Install the `com.unity.robotics.ros-tcp-connector` package from the Package Manager using the Git URL.
3.  Add the `ROSConnection` prefab to your scene. This component manages the connection to ROS 2.
4.  By default, it will try to connect to `127.0.0.1` on port `10000`. We will start a corresponding ROS 2 node to complete the bridge.

## 3. Importing and Configuring the Robot

Unity can't directly import URDFs in the same way Gazebo can. We use the `URDF-Importer` package (another Unity Robotics package) to convert our `humanoid_base.urdf.xacro` file into a Unity prefab.

During import, Unity replaces ROS-style joints with its own **ArticulationBody** components. These are special physics components designed for robotic arms and kinematic chains, providing more stability than standard Rigidbody joints.

*(Placeholder: Image showing the imported humanoid model in the Unity editor with ArticulationBody components visible in the Inspector)*

## 4. Controlling the Robot from ROS 2

We'll create a C# script in Unity that subscribes to a ROS 2 topic and applies the received joint commands to the robot's ArticulationBody components.

**File: `code-examples/module2-the-digital-twin/chapter2.2/unity_project/Scripts/JointSubscriber.cs`**
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointSubscriber : MonoBehaviour
{
    void Start()
    {
        // Subscribe to the "joint_commands" topic
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("joint_commands", OnJointCommand);
    }

    void OnJointCommand(JointStateMsg jointState)
    {
        // Find the corresponding joint in the model
        var joint = this.transform.Find(jointState.name[0]); // Simplified for one joint
        if (joint == null) return;

        var articulationBody = joint.GetComponent<ArticulationBody>();
        if (articulationBody == null) return;

        // Apply the command to the joint's target position
        var drive = articulationBody.xDrive;
        drive.target = (float)(jointState.position[0] * Mathf.Rad2Deg); // Convert radians to degrees
        articulationBody.xDrive = drive;
    }
}
```
*This script would be attached to the root of the robot prefab in Unity.*

On the ROS 2 side, we need a Python script to publish the `JointState` messages that our Unity script is listening for.

**File: `code-examples/module2-the-digital-twin/chapter2.2/ros2_scripts/unity_joint_commander.py`**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class UnityJointCommander(Node):
    def __init__(self):
        super().__init__('unity_joint_commander')
        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.get_logger().info('Unity Joint Commander started.')

    def publish_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['torso_to_left_arm']
        
        position = math.pi / 2 * math.sin(self.get_clock().now().nanoseconds / 1e9)
        msg.position = [position]
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    commander = UnityJointCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Now, when you run your Unity scene and this ROS 2 node, you will see the humanoid's arm moving in your high-fidelity Unity environment, controlled entirely by ROS 2!
