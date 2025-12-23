import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class UnityJointCommander(Node):
    """
    Publishes JointState messages to be consumed by a Unity simulation
    with the ROS-TCP-Connector.
    """
    def __init__(self):
        super().__init__('unity_joint_commander')
        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)
        
        # Publish commands at a rate of 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.get_logger().info('Unity Joint Commander node has been started.')
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def publish_command(self):
        """
        Generates and publishes a JointState message with an oscillating position.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Specify the name of the joint to control.
        # This must match the joint name in the URDF and the GameObject name in Unity.
        msg.name = ['torso_to_left_arm']
        
        # Calculate a smooth oscillating motion using a sine wave.
        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        position = (math.pi / 2.0) * math.sin(elapsed_time) # Oscillates between -pi/2 and +pi/2
        
        # The position array must have the same number of elements as the name array.
        msg.position = [position]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing command for {msg.name[0]}: {position:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    try:
        commander = UnityJointCommander()
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
