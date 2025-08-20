import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarReader:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot
        self.robot_name = self.robot_.getName()
        self.timestep_ = int(self.robot_.getBasicTimeStep())

        try:
            rclpy.init(args=None)
        except:
            pass

        self.node_ = rclpy.create_node("lidar_reader", namespace=f"/{self.robot_name}")
        self.lidar_ = self.robot_.getDevice("lidar")
        self.lidar_.enable(self.timestep_)

        self.publisher = self.node_.create_publisher(
            LaserScan, f"/{self.robot_name}/scan", 10
        )

        self.node_.create_timer(0.1, self.pub_scan)

        self.node_.get_logger().info(
            f"Webots lidar reader for {self.robot_name} is initialized."
        )

    def pub_scan(self):
        if not rclpy.ok():
            return

        ranges = self.lidar_.getRangeImage()

        msg = LaserScan()
        msg.header.stamp = self.node_.get_clock().now().to_msg()
        msg.header.frame_id = "lidar"

        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / (self.lidar_.getHorizontalResolution() - 1)
        msg.scan_time = 1.0 / self.lidar_.getFrequency()
        msg.time_increment = msg.scan_time / self.lidar_.getHorizontalResolution()
        msg.range_min = self.lidar_.getMinRange()
        msg.range_max = self.lidar_.getMaxRange()
        msg.ranges = ranges

        self.publisher.publish(msg)

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)
