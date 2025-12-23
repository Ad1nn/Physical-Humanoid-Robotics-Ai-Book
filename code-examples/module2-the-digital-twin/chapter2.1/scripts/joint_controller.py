import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JointController(Node):
    """
    A simple ROS 2 node that publishes joint trajectory commands to control a robot's arm.
    This is a basic example for demonstrating joint control in a simulation.
    """
    def __init__(self):
        super().__init__('joint_controller')
        # This topic name '/joint_trajectory_controller/joint_trajectory' is a standard
        # convention used by ros2_control for joint trajectory controllers.
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        # Create a timer that fires every second
        self.timer = self.create_timer(1.0, self.publish_command)
        self.get_logger().info('Joint Controller node has been started.')

    def publish_command(self):
        """
        Generates and publishes a JointTrajectory message to create an oscillating motion.
        """
        msg = JointTrajectory()
        msg.joint_names = ['torso_to_right_arm']  # The joint we want to control

        point = JointTrajectoryPoint()
        
        # Calculate a sine wave position based on the current simulation time
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        position = 0.5 * math.sin(current_time_sec) # Oscillate between -0.5 and 0.5 radians
        
        point.positions = [position]
        
        # Set the time for this point to be reached
        time_from_start = Duration()
        time_from_start.sec = 1 # Reach this point in 1 second
        point.time_from_start = time_from_start

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint command for torso_to_right_arm: {position:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    try:
        joint_controller = JointController()
        rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
