import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String, Int32
import random
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}


class DelayedSubscriber(Node):

    def __init__(self):
        super().__init__('delayed_subscriber')
        self.qos_profile = "R10" #default profile
        self.msg = 0
        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher = self.create_publisher(Int32, 'published_messages', qos_profiles[self.qos_profile])  # Publisher to send messages
        self.subscription = self.create_subscription(
            Int32, 'counter_topic',
            self.message_callback,
            qos_profiles[self.qos_profile],
            callback_group=self.sub_cb_group)  # Subscriber to receive messages

    def message_callback(self, msg):
        # Generate a random delay (replace this with your own logic)
        delay_seconds = random.uniform(1.0, 5.0)  # Example: random delay between 0.5 and 2.0 seconds
        delay_seconds = 1.0
        self.get_logger().info(f'Received: {msg.data}, Delay: {delay_seconds:.2f} seconds')
        self.msg = msg
        # Schedule the processing of the message with a timer
        self.timer = self.create_timer(delay_seconds, self.process_message_callback, callback_group=self.timer_cb_group)

    def process_message_callback(self):
        # Process the incoming message
        self.get_logger().info(f'Processing: {self.msg.data}')

        # Publish the incoming message to another topic
        self.publisher.publish(self.msg)
        self.timer.cancel()



def main():
    rclpy.init()

    # Create nodes

    delayed_subscriber_node = DelayedSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(delayed_subscriber_node)

    executor.spin()

    executor.shutdown()

if __name__ == '__main__':
    main()
