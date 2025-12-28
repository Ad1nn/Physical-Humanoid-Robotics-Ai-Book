import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts # Standard service type
from rclpy.executors import MultiThreadedExecutor

# Publisher Node
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('multi_node_publisher')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher Node Started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Publisher: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to topic: "{msg.data}"')
        self.i += 1

# Subscriber Node
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('multi_node_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscriber Node Started, listening to chat_topic')

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribing from topic: "{msg.data}"')

# Service Server Node
class MinimalService(Node):
    def __init__(self):
        super().__init__('multi_node_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service Server Node Started, offering add_two_ints service')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b} -> sum={response.sum}')
        return response

# Service Client Node (will call the service once)
class MinimalClient(Node):
    def __init__(self):
        super().__init__('multi_node_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('Service Client Node Started')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Calling service add_two_ints with a={a}, b={b}')
        return self.future

def main(args=None):
    rclpy.init(args=args)

    # Initialize all nodes
    publisher_node = MinimalPublisher()
    subscriber_node = MinimalSubscriber()
    service_server_node = MinimalService()
    service_client_node = MinimalClient()

    # Create an executor to run all nodes concurrently
    # MultiThreadedExecutor allows nodes to process callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)
    executor.add_node(service_server_node)
    executor.add_node(service_client_node)

    # Send a service request from the client node
    future = service_client_node.send_request(5, 7)

    try:
        # Spin the executor. This will keep the nodes alive and process callbacks.
        # It will also wait for the service client's future to complete.
        while rclpy.ok():
            executor.spin_once() # Process available callbacks
            if future.done():
                try:
                    response = future.result()
                    service_client_node.get_logger().info(
                        f'Result of add_two_ints: for {service_client_node.req.a} + '
                        f'{service_client_node.req.b} = {response.sum}')
                except Exception as e:
                    service_client_node.get_logger().error(f'Service call failed: {e}')
                break # Exit after service call is done and processed
    except KeyboardInterrupt:
        service_client_node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        # Destroy all nodes and shutdown rclpy
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        service_server_node.destroy_node()
        service_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
