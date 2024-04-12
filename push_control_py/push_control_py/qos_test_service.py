from custom_interfaces.srv import TimeMeasurement

import rclpy
from rclpy.node import Node
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Low Latency QoS Profile
low_latency_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,    # Reliable delivery
    history=QoSHistoryPolicy.KEEP_LAST,           # Keep only the last message
    depth=1,                                      # Keep one message in history
)
# High Reliability QoS Profile
high_reliability_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, # Best effort delivery not possible with a service!!!
    history=QoSHistoryPolicy.KEEP_ALL,            # Keep all messages in history
    depth=100,                                     # Keep 10 messages in history
)

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv_low_latency = self.create_service(TimeMeasurement, 'time_measurement_qos_low_latency', self.measurement_callback, qos_profile=low_latency_qos)
        self.srv_high_reliability = self.create_service(TimeMeasurement, 'time_measurement_qos_high_reliability', self.measurement_callback, qos_profile=high_reliability_qos)

    def measurement_callback(self, request, response):
        response.counter_out = request.counter_in #+ request.b
        response.latency = (time.time()-request.timestamp_in)*1000
        #self.get_logger().info(f'Incoming request: {request.counter_in} after: {response.latency}')

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
