import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator # Requires nav2_simple_commander

class GoalPublisher(Node):
    """
    A ROS 2 node to publish a sequence of navigation goals to the Nav2 stack.
    This is a placeholder script for the autonomous navigation project.
    """
    def __init__(self):
        super().__init__('goal_publisher')
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.current_goal_index = 0
        self.goals = []

        # Define a list of goals (PoseStamped)
        # Students will define these based on their Isaac Sim environment
        self.goals.append(self.create_goal(1.0, 0.0, 0.0)) # Example goal 1
        self.goals.append(self.create_goal(1.0, 1.0, 1.57)) # Example goal 2

        self.timer = self.create_timer(5.0, self.publish_next_goal) # Publish every 5 seconds (for demo)
        self.get_logger().info('Goal Publisher node started.')

    def create_goal(self, x, y, yaw):
        """Helper to create a PoseStamped goal message."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal_pose

    def publish_next_goal(self):
        if not self.navigator.isTaskComplete():
            self.get_logger().info('Navigation task not yet complete.')
            return

        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            self.get_logger().info(f"Sending goal {self.current_goal_index + 1}: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}, yaw={math.degrees(math.atan2(goal.pose.orientation.z * 2.0, goal.pose.orientation.w * 2.0)):.2f}")
            self.navigator.goToPose(goal)
            self.current_goal_index += 1
        else:
            self.get_logger().info('All goals sent. Shutting down.')
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        goal_publisher = GoalPublisher()
        # Initial wait to ensure Nav2 is up
        goal_publisher.navigator.waitUntilNav2Active()
        rclpy.spin(goal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
