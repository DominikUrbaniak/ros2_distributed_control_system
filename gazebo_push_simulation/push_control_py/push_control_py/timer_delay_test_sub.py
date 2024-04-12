import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String, Int32
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}



class FinalSubscriber(Node):

    def __init__(self):
        super().__init__('final_subscriber')
        self.qos_profile = "R10" #default profile

        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]
        self.subscription = self.create_subscription(
            Int32,
            'published_messages',
            self.message_callback,
            qos_profiles[self.qos_profile])  # Subscriber to receive published messages

    def message_callback(self, msg):
        # Process the received published message
        self.get_logger().info(f'Final Subscriber Received: {msg.data}')

def main():
    rclpy.init()

    node = FinalSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
