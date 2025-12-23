import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 # For depth camera point cloud data
# import other necessary messages for depth data processing

class DepthProcessor(Node):
    """
    Skeletal ROS 2 node for Challenge 2: Integrate a New Sensor.
    Subscribes to PointCloud2 data from a depth camera and calculates
    the average depth in the scene.
    """
    def __init__(self):
        super().__init__('depth_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points', # Example topic name
            self.depth_callback,
            10
        )
        self.subscription
        self.get_logger().info('Depth Processor node started, subscribing to /depth_camera/points')

    def depth_callback(self, msg: PointCloud2):
        """
        Callback for PointCloud2 messages.
        (Implementation details for processing PointCloud2 would go here)
        For this challenge, implement logic to calculate and log the average depth.
        """
        # Placeholder: In a real scenario, you'd parse msg.data using struct.unpack
        # and msg.fields to get x, y, z coordinates.
        # For simplicity in this challenge, students can implement a basic parsing or
        # focus on the subscription/publishing aspect.
        
        self.get_logger().info('Received PointCloud2 message. Implement processing here for Challenge 2.')
        # Example: calculate average Z-coordinate if it were structured properly
        # avg_depth = ...
        # self.get_logger().info(f'Average depth: {avg_depth:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    try:
        depth_processor = DepthProcessor()
        rclpy.spin(depth_processor)
    except KeyboardInterrupt:
        pass
    finally:
        depth_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
