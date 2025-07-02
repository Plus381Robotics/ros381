import rclpy
from ros381_interfaces.msg import Float2

class EncoderReader:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

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
        self.encoders_pub_ = self.node_.create_publisher(
            Float2, "base_encoders", 10
        )

        self.encoders_ = Float2()
        self.encoders_.float2[0] = 0  # Right passive wheel encoder
        self.encoders_.float2[1] = 0  # Left passive wheel encoder
            
        self.prev_right_pos_ = self.right_encoder_.getValue()
        self.prev_left_pos_ = self.left_encoder_.getValue()
        
        self.node_.get_logger().info("Webots encoder reader is initialized.")

    def pub_encoders(self):
        self.encoders_.float2[0] = (self.right_encoder_.getValue() - self.prev_right_pos_)
        self.encoders_.float2[1] = (self.left_encoder_.getValue() - self.prev_left_pos_)
        
        # Update previous values
        self.prev_right_pos_ = self.right_encoder_.getValue()
        self.prev_left_pos_ = self.left_encoder_.getValue()
        
        # Publish encoders
        self.encoders_pub_.publish(self.encoders_)

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)
        self.pub_encoders()