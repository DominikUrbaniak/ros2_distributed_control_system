import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from custom_interfaces.srv import TimeMeasurement
import time

counter1 = 0
counter2 = 0
start_time = time.time()
class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')

        self.communication_rate = 60.0
        # Create separate callback groups
        self.timer1_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer2_callback_group = MutuallyExclusiveCallbackGroup()
        self.service1_callback_group = MutuallyExclusiveCallbackGroup()
        self.service2_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the timer in the timer callback group
        self.timer1 = self.create_timer(1.0/self.communication_rate, self.timer_callback1, callback_group=self.timer1_callback_group)
        #self.timer2 = self.create_timer(1.0/self.communication_rate, self.timer_callback2, callback_group=self.timer2_callback_group)

        # Create the client in the service callback group
        self.client_low_latency = self.create_client(TimeMeasurement, 'time_measurement_qos_low_latency', callback_group=self.service1_callback_group)
        #self.client_high_reliability = self.create_client(TimeMeasurement, 'time_measurement_qos_high_reliability', callback_group=self.service2_callback_group)

    def timer_callback1(self):
        start_time = time.time()
        if not self.client_low_latency.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service1 not available.')
            return
        global counter1
        counter1 = counter1 + 1
        request = TimeMeasurement.Request()
        request.counter_in = counter1
        request.timestamp_in = time.time()
        #self.get_logger().info(f'Sum (without client call): 30')
        #self.get_logger().info(f'**LL** {request.counter_in} at time {request.timestamp_in-start_time}')
        response = self.client_low_latency.call(request)
        #rclpy.spin_until_future_complete(self, future)
        latency_round_trip = (time.time() - request.timestamp_in)*1000
        computation_latency = time.time() - start_time
        self.get_logger().info(f'**LL** {response.counter_out} - one-way latency: {response.latency}, round-trip latency: {latency_round_trip}, computation latency: {computation_latency}')

    def timer_callback2(self):
        if not self.client_high_reliability.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service2 not available.')
            return
        global counter2
        counter2 = counter2 + 1
        request = TimeMeasurement.Request()
        request.counter_in = counter2
        request.timestamp_in = time.time()
        #self.get_logger().info(f'Sum (without client call): 30')
        #self.get_logger().info(f'**HR** {request.counter_in} at time {request.timestamp_in-start_time}')
        response = self.client_high_reliability.call(request)
        #rclpy.spin_until_future_complete(self, future)
        latency_round_trip = (time.time() - request.timestamp_in)*1000
        self.get_logger().info(f'**HR** {response.counter_out} - one-way latency: {response.latency}, round-trip latency: {latency_round_trip}')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    #executor.add_node(control_node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    executor.shutdown()

if __name__ == '__main__':
    main()
