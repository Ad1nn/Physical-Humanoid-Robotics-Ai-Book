import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrokenListener(Node):
    def __init__(self):
        super().__init__('broken_listener')
        # ERROR: Topic name is incorrect, should match publisher
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    broken_listener = BrokenListener()
    rclpy.spin(broken_listener)
    broken_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
