import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Empty


class ChinchTriggerNode(Node):
    def __init__(self):
        super().__init__("chinch_trigger_node")

        # Publishers
        self.pub_blue = self.create_publisher(Bool, "/blue/chinch_trigger", 10)
        self.pub_yellow = self.create_publisher(Bool, "/yellow/chinch_trigger", 10)

        # Subscriber
        self.sub_chinch = self.create_subscription(
            Empty, "/global_chinch", self.global_chinch_callback, 10
        )

        self.get_logger().info("ChinchTriggerNode initialized.")

    def global_chinch_callback(self, msg):
        bool_msg = Bool()
        bool_msg.data = True

        self.pub_blue.publish(bool_msg)
        self.pub_yellow.publish(bool_msg)

        self.get_logger().info("Published chinch triggers.")
        self.triggered = True


def main(args=None):
    rclpy.init(args=args)
    node = ChinchTriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
