import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
import threading

class CapstoneMainScript(Node):
    """
    Main execution script for the Capstone Project.
    This script simulates sending high-level voice commands and monitors robot feedback.
    In a real system, it would orchestrate the entire VLA pipeline from a top level.
    """
    def __init__(self):
        super().__init__('capstone_main_script')
        self.command_publisher = self.create_publisher(String, 'audio_input', 10) # Simulates audio input
        self.feedback_subscription = self.create_subscription(
            String,
            'robot_feedback',
            self.feedback_callback,
            10
        )
        self.get_logger().info('Capstone Main Script started.')
        self.commands_to_send = [
            "go to the kitchen",
            "find the red cup",
            "bring the red cup to the desk",
            "report current status"
        ]
        self.command_index = 0
        self.timer = self.create_timer(10.0, self.send_next_command) # Send a command every 10 seconds

    def feedback_callback(self, msg: String):
        """
        Callback for robot feedback messages.
        """
        self.get_logger().info(f"Robot Feedback: {msg.data}")

    def send_next_command(self):
        if self.command_index < len(self.commands_to_send):
            command = self.commands_to_send[self.command_index]
            self.get_logger().info(f"Sending command: '{command}'")
            # For demonstration, we directly publish to 'audio_input' which voice_command_node subscribes to.
            # In a real system, this would be actual audio data.
            msg = String() 
            msg.data = command # Simulating audio content as text for simplicity
            self.command_publisher.publish(msg)
            self.command_index += 1
        else:
            self.get_logger().info("All commands sent. Capstone demo complete.")
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CapstoneMainScript()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
