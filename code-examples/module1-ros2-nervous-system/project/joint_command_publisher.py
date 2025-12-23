import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # Standard ROS 2 message type for joint states
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer_period = 0.1 # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Joint Command Publisher Node Started')

        # Define joint names for a 6-DOF arm (from arm_robot.urdf.xacro)
        self.joint_names = [
            'shoulder_yaw_joint',
            'shoulder_pitch_joint',
            'shoulder_roll_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.time = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Example: Make the elbow_pitch_joint oscillate
        # You can expand this logic for more complex movements or target poses
        self.joint_positions[3] = math.sin(self.time) * (math.pi / 4) # Oscillate between -45 and +45 degrees for elbow
        
        # Keep other joints at 0 for simplicity, or add more complex movements
        msg.position = self.joint_positions
        msg.velocity = [] # Optional
        msg.effort = []   # Optional

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')
        self.time += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()
    rclpy.spin(joint_command_publisher)
    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
