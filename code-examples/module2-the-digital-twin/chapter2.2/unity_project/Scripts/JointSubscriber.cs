using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Assuming we use JointState from sensor_msgs

/// <summary>
/// This class subscribes to a ROS topic carrying joint commands and applies them
/// to the corresponding ArticulationBody components of the robot in the Unity scene.
/// </summary>
public class JointSubscriber : MonoBehaviour
{
    // The ROS topic name to subscribe to.
    public string topicName = "joint_commands";

    void Start()
    {
        // Get the ROSConnection instance
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to the specified topic
        ros.Subscribe<JointStateMsg>(topicName, OnJointCommand);
        
        Debug.Log($"Subscribed to topic: {topicName}");
    }

    /// <summary>
    /// Callback function executed when a new JointState message is received.
    /// </summary>
    /// <param name="jointState">The received ROS message.</param>
    void OnJointCommand(JointStateMsg jointState)
    {
        // Iterate through all the joints in the received message
        for (int i = 0; i < jointState.name.Length; i++)
        {
            // Find the GameObject representing the joint in the Unity scene
            // This assumes the joint GameObjects are named the same as in the URDF/ROS message
            Transform jointTransform = this.transform.Find(jointState.name[i]);

            if (jointTransform == null)
            {
                Debug.LogWarning($"Joint '{jointState.name[i]}' not found in the model.");
                continue;
            }

            ArticulationBody articulationBody = jointTransform.GetComponent<ArticulationBody>();
            if (articulationBody == null)
            {
                Debug.LogWarning($"Joint '{jointState.name[i]}' does not have an ArticulationBody component.");
                continue;
            }

            // Get the current drive state of the joint
            var drive = articulationBody.xDrive;
            
            // Set the target position, converting from radians (ROS standard) to degrees (Unity standard)
            drive.target = (float)(jointState.position[i] * Mathf.Rad2Deg);
            
            // Apply the new drive state
            articulationBody.xDrive = drive;
        }
    }
}
