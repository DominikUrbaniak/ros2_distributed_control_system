import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import PoseSensing
import time

counter = 0
communication_rate = 30
class LowLatencyPublisher(Node):
    def __init__(self):
        super().__init__('low_latency_publisher')

        # Define the low latency QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
            history=QoSHistoryPolicy.KEEP_ALL,         # Keep only the last message KEEP_ALL
            depth=100                                      # Keep one message in history 100
        )

        # Create the publisher with the low latency QoS profile
        self.publisher = self.create_publisher(PoseSensing, 'low_latency_topic', qos_profile)

        # Call the publish_message function every second
        self.timer = self.create_timer(1.0/communication_rate, self.publish_message)

    def publish_message(self):
        # Create and publish a message
        msg = PoseSensing()
        global counter
        counter = counter + 1
        msg.pose.position.x = 1.0
        msg.latency_mms = counter
        msg.time_stamp_origin = time.time_ns()
        #msg.data = 'Hello, low latency world!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Message sent at {msg.time_stamp_origin}')

def main(args=None):
    rclpy.init(args=args)
    node = LowLatencyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
