import rclpy
from rclpy.node import Node
from ros381_interfaces.msg import Float3


class EncoderReader:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

        self.ros_node = Node('encoder_reader')
        
        self.ros_node.declare_parameter("d_right", 0.072)
        self.r_right_ = (
            self.ros_node.get_parameter("d_right").get_parameter_value().double_value * 0.5
        )
        self.ros_node.declare_parameter("d_left", 0.072)
        self.r_left_ = (
            self.ros_node.get_parameter("d_left").get_parameter_value().double_value * 0.5
        )

        self.wheel_right_ = self.robot_.getDevice("odometry_wheel_right")
        self.wheel_left_ = self.robot_.getDevice("odometry_wheel_left")

        self.right_encoder_ = self.wheel_right_.getPositionSensor()
        self.left_encoder_ = self.wheel_left_.getPositionSensor()
        self.right_encoder_.enable(int(self.robot_.getBasicTimeStep()))
        self.left_encoder_.enable(int(self.robot_.getBasicTimeStep()))

        try:
            rclpy.init(args=None)
        except:
            pass
        self.node_ = rclpy.create_node("encoder_reader")
        self.encoders_pub_ = self.node_.create_publisher(Float3, "base_encoders", 10)

        self.encoders_ = Float3()
        self.encoders_.float3[0] = 0  # Right passive wheel encoder [m/s]
        self.encoders_.float3[1] = 0  # Left passive wheel encoder [m/s]
        self.encoders_.float3[2] = 0  # Delta time [s]
        self.prev_time_ = self.robot_.getTime()

        self.prev_right_pos_ = self.right_encoder_.getValue()
        self.prev_left_pos_ = self.left_encoder_.getValue()

        self.node_.get_logger().info("Webots encoder reader is initialized.")

    def pub_encoders(self):
        current_time_ = self.robot_.getTime()
        dt_ = current_time_ - self.prev_time_

        if dt_ > 0:
            self.encoders_.float3[0] = (
                (self.right_encoder_.getValue() - self.prev_right_pos_)
                * self.r_right_
                / dt_
            )
            self.encoders_.float3[1] = (
                (self.left_encoder_.getValue() - self.prev_left_pos_)
                * self.r_left_
                / dt_
            )
            self.encoders_.float3[2] = dt_

            # Update previous values
            self.prev_right_pos_ = self.right_encoder_.getValue()
            self.prev_left_pos_ = self.left_encoder_.getValue()
            self.prev_time_ = current_time_

            # Publish encoders
            self.encoders_pub_.publish(self.encoders_)

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)
        self.pub_encoders()
