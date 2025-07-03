import rclpy
from rclpy.node import Node
from ros381_interfaces.msg import Float2


class MotorDriver:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

        try:
            rclpy.init(args=None)
        except:
            pass
        self.node_ = rclpy.create_node("motor_driver")
        self.ros_node = Node("motor_driver")

        self.ros_node.declare_parameter("d_right", 0.07)
        self.r_right_ = (
            self.ros_node.get_parameter("d_right").get_parameter_value().double_value
            * 0.5
        )
        self.ros_node.declare_parameter("d_left", 0.07)
        self.r_left_ = (
            self.ros_node.get_parameter("d_left").get_parameter_value().double_value
            * 0.5
        )

        self.motor_right_ = self.robot_.getDevice("wheel_right")
        self.motor_left_ = self.robot_.getDevice("wheel_left")

        self.motor_right_.setPosition(float("inf"))
        self.motor_right_.setVelocity(0)
        self.motor_left_.setPosition(float("inf"))
        self.motor_left_.setVelocity(0)

        self.cmd_sub_ = self.node_.create_subscription(
            Float2, "motor_cmd", self.cmd_vel_callback, 1
        )

        self.w_right_ = 0  # Right motor velocity   [rad/s]
        self.w_left_ = 0  # Left motor velocity    [rad/s]

        self.node_.get_logger().info("Webots motor driver is initialized.")

    def cmd_vel_callback(self, motor_cmd):
        self.w_right_ = motor_cmd.float2[0] / self.r_right_
        self.w_left_ = motor_cmd.float2[1] / self.r_left_

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)

        self.motor_right_.setVelocity(self.w_right_)
        self.motor_left_.setVelocity(self.w_left_)
        self.node_.get_logger().info("Motor velocity commands: ( " + str(self.w_right_) + ", " + str(self.w_left_) + ")")
