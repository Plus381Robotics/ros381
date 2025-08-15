import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ros381_interfaces.msg import Float3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow


class LidarPlot(Node):
    def __init__(self):
        super().__init__("lidar_plot")
        self.declare_parameter("robot_name", "ros381")
        self.robot_ = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.scan_sub_ = self.create_subscription(
            LaserScan, f"/{self.robot_}/scan", self.plot_scan, 10
        )
        self.odom_sub_ = self.create_subscription(
            Odometry, f"/{self.robot_}/odom", self.set_odom, 10
        )
        self.pose_offset_sub_ = self.create_subscription(
            Float3, f"/{self.robot_}/pose_offset", self.offset_pose, 10
        )
        self.robot_x_ = 0
        self.robot_y_ = 0
        self.robot_phi_ = 0

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [], s=4)
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.0, 1.0)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("LIDAR Scan Data (XY Coordinates)")
        self.ax.grid(True)

        self.robot_arrow = Arrow(0, 0, 0.4, 0, width=0.2, color="red")
        self.ax.add_patch(self.robot_arrow)
        plt.tight_layout()
        plt.show(block=False)
        self.once_ = True

    def offset_pose(self, msg):
        self.robot_x_ += msg.float3[0]
        self.robot_y_ += msg.float3[1]
        self.robot_phi_ += msg.float3[2]

    def set_odom(self, msg):
        self.robot_x_ = msg.pose.pose.position.x
        self.robot_y_ = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_phi_ = np.arctan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def plot_scan(self, msg):
        self.robot_arrow.remove()  # Remove old arrow
        arrow_length = 0.2
        dx = arrow_length * np.cos(self.robot_phi_)
        dy = arrow_length * np.sin(self.robot_phi_)
        self.robot_arrow = Arrow(
            self.robot_x_, self.robot_y_, dx, dy, width=0.1, color="red"
        )
        self.ax.add_patch(self.robot_arrow)
        angles = np.arange(
            msg.angle_min, msg.angle_max + msg.angle_increment / 2, msg.angle_increment
        )
        angles = angles[: len(msg.ranges)]
        ranges = np.array(msg.ranges)

        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = angles[valid_mask]  # Don't modify the angles yet
        ranges = ranges[valid_mask]

        x_robot = ranges * np.cos(angles)
        y_robot = ranges * np.sin(angles)

        cos_phi = np.cos(self.robot_phi_)
        sin_phi = np.sin(self.robot_phi_)
        x_world = x_robot * cos_phi + y_robot * sin_phi + self.robot_x_
        y_world = x_robot * sin_phi - y_robot * cos_phi + self.robot_y_
        self.scatter.set_offsets(np.column_stack((x_world, y_world)))
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    lidar_plot_ = LidarPlot()
    rclpy.spin(lidar_plot_)
    plt.close(all)
    lidar_plot_.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
