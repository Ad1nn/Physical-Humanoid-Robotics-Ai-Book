import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger # For simple request/response
from sensor_msgs.msg import JointState
import math

# Define some preset joint positions
POSE_PRESETS = {
    "rest": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "wave": [0.0, -0.5, 1.0, -0.5, 0.0, 0.0], # Example values
    "point": [0.0, -1.0, 0.0, 0.0, 0.0, 0.0], # Example values
}

class ArmPoseService(Node):
    def __init__(self):
        super().__init__('arm_pose_service')
        self.srv = self.create_service(Trigger, 'set_arm_pose', self.set_arm_pose_callback)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Arm Pose Service Node Started, offering set_arm_pose service')

        self.joint_names = [
            'shoulder_yaw_joint',
            'shoulder_pitch_joint',
            'shoulder_roll_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

    def set_arm_pose_callback(self, request, response):
        pose_name = request.message.lower() # Assuming request.message contains the pose name
        
        if pose_name in POSE_PRESETS:
            target_positions = POSE_PRESETS[pose_name]
            
            # Create and publish JointState message for the target pose
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.joint_names
            joint_msg.position = target_positions
            joint_msg.velocity = [0.0] * len(self.joint_names) # Assuming no velocity commands
            joint_msg.effort = [0.0] * len(self.joint_names)   # Assuming no effort commands

            self.joint_publisher.publish(joint_msg)
            self.get_logger().info(f'Set arm pose to "{pose_name}". Publishing: {target_positions}')
            
            response.success = True
            response.message = f'Successfully set arm pose to "{pose_name}"'
        else:
            response.success = False
            response.message = f'Pose preset "{pose_name}" not found.'
            self.get_logger().warn(f'Attempted to set unknown arm pose: "{pose_name}"')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    arm_pose_service = ArmPoseService()
    rclpy.spin(arm_pose_service)
    arm_pose_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
