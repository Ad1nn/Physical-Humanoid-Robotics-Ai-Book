import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarProcessor(Node):
    """
    A ROS 2 node that subscribes to LaserScan messages and processes them
    to find the closest obstacle.
    """
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar_scan',  # Topic name from URDF Gazebo sensor plugin
            self.lidar_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('LiDAR Processor node started, subscribing to /lidar_scan')

    def lidar_callback(self, msg: LaserScan):
        """
        Callback function for processing incoming LaserScan messages.
        It iterates through the ranges and identifies the closest obstacle,
        then logs its distance and angle.
        """
        closest_range = float('inf')
        closest_angle = 0.0

        # Iterate through the ranges data
        for i, range_val in enumerate(msg.ranges):
            # Ensure the range value is valid (not infinity or NaN and within min/max)
            if msg.range_min <= range_val <= msg.range_max and not math.isinf(range_val):
                if range_val < closest_range:
                    closest_range = range_val
                    # Calculate the angle of the closest obstacle
                    angle = msg.angle_min + i * msg.angle_increment
                    closest_angle = math.degrees(angle)

        if closest_range != float('inf'):
            self.get_logger().info(f'Closest obstacle: {closest_range:.2f} m at {closest_angle:.2f} degrees')
        else:
            self.get_logger().info('No valid obstacles detected within range.')

def main(args=None):
    rclpy.init(args=args)
    try:
        lidar_processor = LidarProcessor()
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        # Allow the node to exit cleanly on Ctrl+C
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
