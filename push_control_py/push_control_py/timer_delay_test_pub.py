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

class CounterPublisher(Node):

    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher = self.create_publisher(Int32, 'counter_topic', qos_profiles[qos_profile])  # Publisher to publish counter
        self.timer = self.create_timer(0.5, self.publish_counter)
        self.counter = 0

    def publish_counter(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1



def main():
    rclpy.init()

    # Create nodes
    node = CounterPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
