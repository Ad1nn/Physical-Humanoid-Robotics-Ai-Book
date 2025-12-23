import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class VslamEvaluator(Node):
    """
    A ROS 2 node that subscribes to VSLAM odometry/pose topics and evaluates its output.
    This script can be extended to compare estimated pose with ground truth (if available in simulation).
    """
    def __init__(self):
        super().__init__('vslam_evaluator')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry', # Topic from isaac_ros_visual_slam
            self.odometry_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/slam_pose', # PoseStamped topic
            self.pose_callback,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('VSLAM Evaluator node started.')

    def odometry_callback(self, msg: Odometry):
        """
        Callback for Odometry messages from VSLAM.
        """
        self.get_logger().debug(f"Received Odometry: Pos({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})")
        # You can add more detailed evaluation logic here, e.g.,
        # comparing to ground truth, calculating drift.

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for PoseStamped messages from VSLAM.
        """
        self.get_logger().info(f"Received Pose: {msg.header.frame_id} -> {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}")
        # Example: Try to lookup a transform
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'base_link', # Source frame (robot's base)
                now
            )
            self.get_logger().debug(f"Transform from map to base_link: {transform.transform.translation.x:.2f}, {transform.transform.translation.y:.2f}, {transform.transform.translation.z:.2f}")
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform map to base_link: {ex}')

def main(args=None):
    rclpy.init(args=args)
    try:
        vslam_evaluator = VslamEvaluator()
        rclpy.spin(vslam_evaluator)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
