import rclpy
from omni_1_interfaces.msg import MotorCommandArray

WHEEL_RADIUS = 0.035


class MotorDriver:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

        self.motor_right_ = self.robot_.getDevice("wheel_right")
        self.motor_left_ = self.robot_.getDevice("wheel_right")

        self.motor_right_.setPosition(float("inf"))
        self.motor_right_.setVelocity(0)
        self.motor_left_.setPosition(float("inf"))
        self.motor_left_.setVelocity(0)

        self.target_motor_cmd_ = MotorCommandArray()

        rclpy.init(args=None)
        self.node_ = rclpy.create_node("motor_driver")
        self.cmd_sub_ = self.node_.create_subscription(
            MotorCommandArray, "motor_cmd", self.cmd_vel_callback, 1
        )

        self.motor_right_vel_ = 0
        self.motor_left_vel_ = 0

        self.node_.get_logger().info("Webots motor driver is initialized.")

    def cmd_vel_callback(self, motor_cmd):
        if len(motor_cmd.motor_command) == 2:
            self.motor_right_vel_ = motor_cmd.motor_command[0]
            self.motor_left_vel_ = motor_cmd.motor_command[1]
        else:
            self.node_.get_logger().info("Recieved invalid motor command.")

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)

        self.motor_right_.setVelocity(self.motor_right_vel_)
        self.motor_left_.setVelocity(self.motor_left_vel_)
