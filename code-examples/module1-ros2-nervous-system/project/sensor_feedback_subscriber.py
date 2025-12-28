import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # Standard ROS 2 message type for joint states

class SensorFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_feedback_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', # Subscribing to the same topic the publisher publishes to
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Sensor Feedback Subscriber Node Started')

    def listener_callback(self, msg):
        # In a real scenario, this would represent receiving actual joint sensor data
        # For this example, we'll just print the received joint positions
        self.get_logger().info(f'Received JointState feedback:')
        for i in range(len(msg.name)):
            self.get_logger().info(f'  Joint: {msg.name[i]}, Position: {msg.position[i]:.4f}')
        
        # Here you could add logic to process the feedback, e.g.,
        # - Check if actual position matches commanded position
        # - Perform control loop calculations
        # - Detect anomalies

def main(args=None):
    rclpy.init(args=args)
    sensor_feedback_subscriber = SensorFeedbackSubscriber()
    rclpy.spin(sensor_feedback_subscriber)
    sensor_feedback_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
