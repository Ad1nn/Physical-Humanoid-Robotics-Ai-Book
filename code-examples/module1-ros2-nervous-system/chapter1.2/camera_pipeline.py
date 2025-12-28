import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Standard ROS 2 message type for images
from cv_bridge import CvBridge # Library to convert between ROS 2 Image messages and OpenCV images
import cv2 # OpenCV library for image processing
import numpy as np # For array manipulation

# Publisher Node: Simulates a camera publishing image frames
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer_period = 1.0 / 10.0  # Publish at 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info('Camera Publisher Node Started')

    def timer_callback(self):
        # Simulate capturing an image (e.g., a simple colored square)
        # In a real scenario, this would come from a camera sensor
        img_data = np.zeros((480, 640, 3), dtype=np.uint8) # Create a black image
        color = (255, 0, 0) # Blue color
        cv2.rectangle(img_data, (100, 100), (540, 380), color, -1) # Draw a blue rectangle

        # Convert OpenCV image to ROS 2 Image message
        img_msg = self.bridge.cv2_to_imgmsg(img_data, encoding="bgr8")
        self.publisher_.publish(img_msg)
        self.get_logger().info('Publishing simulated camera frame')

# Subscriber Node: Processes image frames
class ImageProcessorSubscriber(Node):
    def __init__(self):
        super().__init__('image_processor_subscriber')
        # Subscribe to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info('Image Processor Subscriber Node Started, listening for image_raw')

    def listener_callback(self, img_msg):
        self.get_logger().info('Receiving camera frame for processing')
        try:
            # Convert ROS 2 Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

            # Perform a simple image processing operation (e.g., convert to grayscale)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Log a message to indicate processing
            self.get_logger().info(f'Processed image: converted to grayscale. Image shape: {gray_image.shape}')

            # In a real application, you might save, display, or further analyze the image
            # For this example, we'll just log
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    image_processor_subscriber = ImageProcessorSubscriber()

    # Use a MultiThreadedExecutor to run both nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(camera_publisher)
    executor.add_node(image_processor_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        image_processor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
