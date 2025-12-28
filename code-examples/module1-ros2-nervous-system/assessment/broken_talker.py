import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrokenTalker(Node):
    def __init__(self):
        super().__init__('broken_talker')
        # ERROR: Topic name is incorrect, should match subscriber
        self.publisher_ = self.create_publisher(String, 'wrong_topic', 10) 
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from BrokenTalker: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    broken_talker = BrokenTalker()
    rclpy.spin(broken_talker)
    broken_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
