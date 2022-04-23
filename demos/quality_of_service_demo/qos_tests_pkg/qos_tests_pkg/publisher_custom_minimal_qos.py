import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)
        # create the publisher object
        #  create_publisher(msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None)
        # INFO: https://docs.ros2.org/foxy/api/rclpy/api/node.html

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

        # This is the Unique id for each of the messages that will be sent
        self.msgs_id = 0
        #self.current_time = self.get_clock().now()
        self.current_time_s = 0
        self.current_time_ns = 0
        # define the timer period for 0.5 seconds
        timer_period = 0.5
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seeconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def incompatible_qos_clb(self, event):
        """
        This is the callback that will be executed when the Event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("A subscriber is asking for an INCOMPATIBLE QoS Triggered!!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def timer_callback(self):
        # Here we have the callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        dds_msg_str = str(self.msgs_id)+":"+time_str
        msg.data = dds_msg_str
        # Publish the message to the topic
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)

        self.msgs_id += 1


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-reliability',
        type=str,
        choices=['best_effort', 'reliable'],
        help='Select Policy for reliability, use ros2 run dds_tests_pkg publisher_dds_custom_qos_exe -reliability best_effort|reliable')
    return parser


def main(args=None):

    # Lets parse the arguments
    parser = get_parser()
    parsed_args = parser.parse_args()

    # Configuration variables
    reliability = parsed_args.reliability
    print(reliability)
    qos_profile_publisher = QoSProfile(depth=10)

    # Options  QoSDurabilityPolicy.VOLATILE, QoSDurabilityPolicy.TRANSIENT_LOCAL,
    qos_profile_publisher.durability = QoSDurabilityPolicy.VOLATILE

    qos_profile_publisher.deadline = Duration(seconds=2)

    # Options QoSLivelinessPolicy.MANUAL_BY_TOPIC, QoSLivelinessPolicy.AUTOMATIC
    qos_profile_publisher.liveliness = QoSLivelinessPolicy.AUTOMATIC

    qos_profile_publisher.liveliness_lease_duration = Duration(seconds=2)

    # Options: QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT
    if reliability == "reliable":
        qos_profile_publisher.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT

    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(pub_qos_obj)
    # Explicity destroy the node
    pub_qos_obj.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()

