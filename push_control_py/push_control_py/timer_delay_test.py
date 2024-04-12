import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String, Int32
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}

qos_profile = "R10" #default profile

if len(sys.argv)>1:
    qos_profile = sys.argv[1]

class DelayedSubscriber(Node):

    def __init__(self):
        super().__init__('delayed_subscriber')
        self.publisher = self.create_publisher(Int32, 'published_messages', qos_profiles[qos_profile])  # Publisher to send messages
        self.subscription = self.create_subscription(
            Int32,
            'counter_topic',
            self.message_callback,
            qos_profiles[qos_profile])  # Subscriber to receive messages

    def message_callback(self, msg):
        # Generate a random delay (replace this with your own logic)
        delay_seconds = random.uniform(0.1, 2.0)  # Example: random delay between 0.5 and 2.0 seconds
        self.get_logger().info(f'Received: {msg.data}, Delay: {delay_seconds:.2f} seconds')

        # Schedule the processing of the message with a timer
        self.create_timer(delay_seconds, self.process_message_callback, msg)

    def process_message_callback(self, msg):
        # Process the incoming message
        self.get_logger().info(f'Processing: {msg.data}')

        # Publish the incoming message to another topic
        self.publisher.publish(msg)

class CounterPublisher(Node):

    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher = self.create_publisher(Int32, 'counter_topic', qos_profiles[qos_profile])  # Publisher to publish counter
        self.timer = self.create_timer(0.5, self.publish_counter)  # Publish at 100Hz (0.01 seconds interval)
        self.counter = 0

    def publish_counter(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

class FinalSubscriber(Node):

    def __init__(self):
        super().__init__('final_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'published_messages',
            self.message_callback,
            qos_profiles[qos_profile])  # Subscriber to receive published messages

    def message_callback(self, msg):
        # Process the received published message
        self.get_logger().info(f'Final Subscriber Received: {msg.data}')

def main():
    rclpy.init()

    # Create nodes
    counter_publisher_node = CounterPublisher()
    delayed_subscriber_node = DelayedSubscriber()
    final_subscriber_node = FinalSubscriber()

    # Spin all nodes
    rclpy.spin(counter_publisher_node)
    rclpy.spin(delayed_subscriber_node)
    rclpy.spin(final_subscriber_node)

if __name__ == '__main__':
    main()
