import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import time
import sys

class NaceopCommandPublisher(Node):
    def __init__(self):
        super().__init__('naceop_command_publisher')
        qos_profile_str = QoSProfile(depth=1)
        qos_profile_str.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_str.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile_str.reliability = QoSReliabilityPolicy.RELIABLE 
        self.naceop_pub = self.create_publisher(String, '/naceop', qos_profile_str)
        # Subscriber to listen for movement commands
        self.nacedone_sub = self.create_subscription(String, '/nacedone', self.nacedone_callback, qos_profile_str)
        self.nacedone_pub = self.create_publisher(String, '/nacedone', qos_profile_str)
        msg = String()
        msg.data = "notdone"
        self.nacedone_pub.publish(msg)
        self.get_logger().info("Published 'notdone' to /nacedone")
        # Publish "left" to /naceop
        msg = String()
        msg.data = sys.argv[1]
        self.naceop_pub.publish(msg)
        self.get_logger().info(f"Published '{sys.argv[1]}' to /naceop")

    def nacedone_callback(self, msg):
        command = msg.data.lower()
        if command != "notdone":
            current_time = time.time()
            message_time = float(command)
            if current_time - message_time > 1:
                self.get_logger().warn("Received an old message; ignoring it.")
                return
            self.get_logger().info(f"Move done")
            self.get_logger().info("Shutting down the node...")
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = NaceopCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        None
    except SystemExit:
        None
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
