import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import PoseSensing
import time
import numpy as np

communication_rate = 120
communication_duration = 120
n_data_points = int(communication_rate*communication_duration)
latencies = np.zeros([n_data_points,2])
counter = 0

class LowLatencySubscriber(Node):
    def __init__(self):
        super().__init__('low_latency_subscriber')

        # Define the low latency QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery
            history=QoSHistoryPolicy.KEEP_ALL,         # Keep only the last message
            depth=100                                      # Keep one message in history
        )

        # Create the subscriber with the low latency QoS profile
        self.subscriber = self.create_subscription(
            PoseSensing,
            'low_latency_topic',
            self.callback,
            qos_profile
        )

    def callback(self, msg):
        #time_received = time.time_ns()
        latency = time.time_ns() - msg.time_stamp_origin
        global counter
        if counter < n_data_points:
            global latencies
            latencies[counter,:] = [msg.latency_mms, latency]
        else:
            np.savetxt('docs/data/latency_measurement.csv', latencies, delimiter=',')
            self.get_logger().info(f'Successful measurement!')
            self.cleanup()

        counter = counter + 1
        self.get_logger().info(f'Message received after {latency}')

    def cleanup(self):
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LowLatencySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
