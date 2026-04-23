import rclpy
from rclpy.node import Node
from ros381_interfaces.msg import Float3
from ros381_interfaces.srv import UpdatePose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

R_RIGHT = 0.072 * 0.5
R_LEFT = 0.072 * 0.5
L = 0.297
L_recip = 1 / L


class OdometryW:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot
        self.robot_name = self.robot_.getName()

        try:
            rclpy.init(args=None)
        except:
            pass

        self.node_ = rclpy.create_node("odometry", namespace=f"/{self.robot_name}")
        self.encoders_sub_ = self.node_.create_subscription(
            Float3,
            f"/{self.robot_name}/base_encoders",
            self.encoders_callback,
            10,
        )
        self.odometry_pub_ = self.node_.create_publisher(
            Odometry, f"/{self.robot_name}/odom", 10
        )
        self.node_.get_logger().info(
            f"Webots odometry node for {self.robot_name} is initialized."
        )
        self.odometry_msg_ = Odometry()
        self.odometry_msg_.pose.pose.position.x = 0.0
        self.odometry_msg_.pose.pose.position.y = 0.0
        self.odometry_msg_.pose.pose.position.z = 0.0
        self.odometry_msg_.pose.pose.orientation.x = 0.0
        self.odometry_msg_.pose.pose.orientation.y = 0.0
        self.odometry_msg_.pose.pose.orientation.z = 0.0
        self.odometry_msg_.pose.pose.orientation.w = 0.0
        self.odometry_msg_.twist.twist.linear.x = 0.0
        self.odometry_msg_.twist.twist.linear.y = 0.0
        self.odometry_msg_.twist.twist.linear.z = 0.0
        self.odometry_msg_.twist.twist.angular.x = 0.0
        self.odometry_msg_.twist.twist.angular.y = 0.0
        self.odometry_msg_.twist.twist.angular.z = 0.0
        self.odometry_msg_.header.stamp = self.node_.get_clock().now().to_msg()

        self.x_base_ = 0.0
        self.y_base_ = 0.0
        self.phi_base_ = 0.0
        
        self.update_srv_ = self.node_.create_service(
            UpdatePose,
            'update_pose',
            self.callback_update_pose
        )
        self.odom_init_ = False;
    
    
    def callback_update_pose(self, request, response):
        response.success = False

        update_x = (request.type // 100) % 10
        update_y = (request.type // 10) % 10
        update_phi = (request.type // 1) % 10

        if update_x:
            self.x_base_ = request.x
        if update_y:
            self.y_base_ = request.y
        if update_phi:
            self.phi_base_ = request.phi

        self.node_.get_logger().info(
            f"New pose:\nx = {self.x_base_:.2f} mm\ny = {self.y_base_:.2f} mm\nphi = {self.phi_base_:.2f} rad"
        )

        response.success = bool(update_x or update_y or update_phi)
        return response

    def encoders_callback(self, msg):
        if msg.float3[2] > 0.0 and not math.isnan(msg.float3[0]) and not math.isnan(msg.float3[1]) and not math.isnan(msg.float3[2]):
            self.calculate_odometry(msg.float3[0], msg.float3[1], msg.float3[2])

    def calculate_odometry(self, v_right, v_left, dt):
        if self.odom_init_:
            # 1. izracunaj odometriju
            self.odometry_msg_.header.stamp = self.node_.get_clock().now().to_msg()
            v_base = (v_right + v_left) * 0.5;
            w_base = (v_right - v_left) * L_recip;
            
            self.node_.get_logger().info(
                f"w = {w_base:.2f}"
            )
            mid_angle = self.phi_base_ + w_base * dt * 0.5;
            
            self.x_base_ += v_base * math.cos(mid_angle) * dt;
            self.y_base_ += v_base * math.sin(mid_angle) * dt;
            self.phi_base_ += w_base * dt;
            diff = 2 * math.pi
            while self.phi_base_ > math.pi:
                self.phi_base_ -= diff
            while self.phi_base_ < -math.pi:
                self.phi_base_ += diff
            
            # 2. pub odometriju
            q = Quaternion()
            cy = math.cos(self.phi_base_ * 0.5)
            sy = math.sin(self.phi_base_ * 0.5)
            self.odometry_msg_.pose.pose.position.x = self.x_base_
            self.odometry_msg_.pose.pose.position.y = self.y_base_
            self.odometry_msg_.pose.pose.orientation.z = sy
            self.odometry_msg_.pose.pose.orientation.w = cy
            self.odometry_msg_.twist.twist.linear.x = v_base
            self.odometry_msg_.twist.twist.angular.z = w_base
            self.odometry_pub_.publish(self.odometry_msg_)
        else:
            self.odom_init_ = True

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)
