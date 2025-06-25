import rclpy
from ros381_interfaces.msg import Float2

WHEEL_RADIUS = 0.035

class MotorDriver:
    def __init__(self, webots_node, properties):
        self.robot_ = webots_node.robot

        self.motor_right_ = self.robot_.getDevice("wheel_right")
        self.motor_left_ = self.robot_.getDevice("wheel_left")

        self.motor_right_.setPosition(float("inf"))
        self.motor_right_.setVelocity(0)
        self.motor_left_.setPosition(float("inf"))
        self.motor_left_.setVelocity(0)

        self.target_motor_cmd_ = Float2()

        rclpy.init(args=None)
        self.node_ = rclpy.create_node("motor_driver")
        self.cmd_sub_ = self.node_.create_subscription(
            Float2, "motor_cmd", self.cmd_vel_callback, 1
        )

        self.motor_vel_ = Float2()
        self.motor_vel_[0] = 0  # Right motor velocity
        self.motor_vel_[1] = 0  # Left motor velocity

        self.node_.get_logger().info("Webots motor driver is initialized.")

    def cmd_vel_callback(self, motor_cmd):
        self.motor_vel_[0] = motor_cmd[0]
        self.motor_vel_[1] = motor_cmd[1]

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)

        self.motor_right_.setVelocity(self.motor_vel_[0])
        self.motor_left_.setVelocity(self.motor_vel_[1])

# def main(args=None):
#     # Empty, as webots should instantiate __init__
#     pass

# def create_plugin(webots_node, properties):
#     return MotorDriver(webots_node, properties)