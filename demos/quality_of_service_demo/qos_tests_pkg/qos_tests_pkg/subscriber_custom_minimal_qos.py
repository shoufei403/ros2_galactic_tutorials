import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import Quality of Service library, to set the correct profile and reliability.
from rclpy.qos import ReliabilityPolicy, QoSProfile


class SubscriberQoS(Node):

    def __init__(self):

        super().__init__('subscriber_qos_obj')

        # create the subscriber object
        self.subscriber = self.create_subscription(
            String,
            '/qos_test',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def listener_callback(self, msg):
        self.get_logger().info("Data Received ="+str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS()
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

