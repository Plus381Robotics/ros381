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
        self.odom_set_ = False
        self.cur_time_ = self.get_clock().now().nanoseconds * 1e-9
        self.prev_time_ = self.cur_time_

    def offset_pose(self, msg):
        self.robot_x_ += msg.float3[0]
        self.robot_y_ += msg.float3[1]
        self.robot_phi_ += msg.float3[2]

    def set_odom(self, msg):
        self.odom_set_ = True
        self.robot_x_ = msg.pose.pose.position.x
        self.robot_y_ = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_phi_ = np.arctan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )
        self.vx_ = msg.twist.twist.linear.x
        self.wz_ = msg.twist.twist.angular.z

    # def plot_scan(self, msg):
    #     if self.odom_set_:
    #         self.cur_time_ = self.get_clock().now().nanoseconds * 1e-9
    #         self.robot_arrow.remove()
    #         arrow_length = 0.2
    #         dx = arrow_length * np.cos(self.robot_phi_)
    #         dy = arrow_length * np.sin(self.robot_phi_)
    #         self.robot_arrow = Arrow(
    #             self.robot_x_, self.robot_y_, dx, dy, width=0.1, color="red"
    #         )
    #         self.ax.add_patch(self.robot_arrow)
    #         angles = np.arange(
    #             msg.angle_min,
    #             msg.angle_max + msg.angle_increment / 2,
    #             msg.angle_increment,
    #         )
    #         angles = angles[: len(msg.ranges)]
    #         ranges = np.array(msg.ranges)

    #         valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
    #         angles = angles[valid_mask]
    #         ranges = ranges[valid_mask]

    #         timestamps = np.linspace(self.cur_time_ - self.prev_time_, 0, len(angles))
    #         delta_phi = self.wz_ * timestamps
    #         delta_x = self.vx_ * timestamps * np.cos(delta_phi)
    #         delta_y = self.vx_ * timestamps * np.sin(delta_phi)

    #         x_robot = ranges * np.cos(angles + delta_phi) + delta_x
    #         y_robot = ranges * np.sin(angles + delta_phi) + delta_y

    #         cos_phi = np.cos(self.robot_phi_)
    #         sin_phi = np.sin(self.robot_phi_)
    #         x_world = x_robot * cos_phi + y_robot * sin_phi + self.robot_x_
    #         y_world = x_robot * sin_phi - y_robot * cos_phi + self.robot_y_
    #         self.scatter.set_offsets(np.column_stack((x_world, y_world)))
    #         self.fig.canvas.draw()
    #         self.fig.canvas.flush_events()
    #         self.prev_time_ = self.cur_time_
    def plot_scan(self, msg):
        if self.odom_set_:
            # Update robot arrow (unchanged)
            self.robot_arrow.remove()
            arrow_length = 0.2
            dx = arrow_length * np.cos(self.robot_phi_)
            dy = arrow_length * np.sin(self.robot_phi_)
            self.robot_arrow = Arrow(self.robot_x_, self.robot_y_, dx, dy, 
                                   width=0.1, color='red')
            self.ax.add_patch(self.robot_arrow)

            # Get scan parameters
            scan_duration = 0.02
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            ranges = np.array(msg.ranges)
            
            # Filter invalid measurements
            valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
            angles = angles[valid_mask]
            ranges = ranges[valid_mask]
            
            # Time offsets (first point is oldest)
            timestamps = np.linspace(scan_duration, 0, len(angles))
            
            # Calculate motion path during scan
            x_world = np.zeros_like(ranges)
            y_world = np.zeros_like(ranges)
            
            for i, (angle, r, t) in enumerate(zip(angles, ranges, timestamps)):
                # Calculate robot pose at measurement time
                frac = t / scan_duration
                current_phi = self.robot_phi_ - self.wz_ * t  # Current yaw at measurement
                current_x = self.robot_x_ - self.vx_ * t * np.cos(current_phi)
                current_y = self.robot_y_ - self.vx_ * t * np.sin(current_phi)
                
                # Convert to world coordinates
                x_robot = r * np.cos(angle)
                y_robot = r * np.sin(angle)
                
                # Transform using pose at measurement time
                cos_phi = np.cos(current_phi)
                sin_phi = np.sin(current_phi)
                x_world[i] = current_x + x_robot * cos_phi + y_robot * sin_phi
                y_world[i] = current_y + x_robot * sin_phi - y_robot * cos_phi
            
            # Update plot
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
