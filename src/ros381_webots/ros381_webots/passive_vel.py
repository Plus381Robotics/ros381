import rclpy
from ros381_interfaces.msg import Float2

WHEEL_RADIUS = 0.036
WHEEL_DISTANCE = 0.297

class PassiveVel:
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
        self.node_ = rclpy.create_node("passive_vel")
        self.vel_pub_ = self.node_.create_publisher(
            Float2, "passive_vel", 10
        )

        self.passive_vel_ = Float2()
        self.passive_vel_.float2[0] = 0  # Right passive wheel velocity
        self.passive_vel_.float2[1] = 0  # Left passive wheel velocity
        self.prev_time_ = self.robot_.getTime()
            
        self.prev_right_pos_ = self.right_encoder_.getValue()
        self.prev_left_pos_ = self.left_encoder_.getValue()
        
        self.node_.get_logger().info("Webots passive velocity reader is initialized.")

    def pub_passive_vel(self):
        current_time = self.robot_.getTime()
        dt = current_time - self.prev_time_

        if dt > 0:
            # Get current positions and calculate velocities (rad/s)
            self.passive_vel_.float2[0] = (self.right_encoder_.getValue() - self.prev_right_pos_) / dt
            self.passive_vel_.float2[1] = (self.left_encoder_.getValue() - self.prev_left_pos_) / dt
            
            # Update previous values
            self.prev_right_pos_ = self.right_encoder_.getValue()
            self.prev_left_pos_ = self.left_encoder_.getValue()
            self.prev_time_ = current_time
            
            # Publish the velocities
            self.vel_pub_.publish(self.passive_vel_)

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)
        self.pub_passive_vel()