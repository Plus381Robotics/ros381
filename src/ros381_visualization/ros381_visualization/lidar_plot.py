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
        self.ax.grid(True)

        self.robot_arrow = Arrow(0, 0, 0.4, 0, width=0.2, color="red")
        self.ax.add_patch(self.robot_arrow)
        plt.tight_layout()
        plt.show(block=False)
        self.odom_set_ = False
        self.create_grid_map(3.0, 2.0, 0.05)

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

    def plot_scan(self, msg):
        if not self.odom_set_:
            return

        self.robot_arrow.remove()
        arrow_length = 0.2
        dx = arrow_length * np.cos(self.robot_phi_)
        dy = arrow_length * np.sin(self.robot_phi_)
        self.robot_arrow = Arrow(
            self.robot_x_, self.robot_y_, dx, dy, width=0.1, color="red"
        )
        self.ax.add_patch(self.robot_arrow)

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = angles[valid_mask]
        ranges = ranges[valid_mask]

        scan_duration = msg.scan_time
        timestamps = np.linspace(scan_duration, 0.0, len(angles))

        x_world = np.zeros_like(ranges)
        y_world = np.zeros_like(ranges)

        for i, (angle, r, t) in enumerate(zip(angles, ranges, timestamps)):
            phi_t = self.robot_phi_ - self.wz_ * t
            x_t = self.robot_x_ - self.vx_ * t * np.cos(phi_t)
            y_t = self.robot_y_ - self.vx_ * t * np.sin(phi_t)

            x_world[i] = x_t + r * np.cos(angle - phi_t)
            y_world[i] = y_t - r * np.sin(angle - phi_t)

        self.scatter.set_offsets(np.column_stack((x_world, y_world)))
        self.update_map(x_world, y_world)
        self.grid_img.set_data(self.grid.T)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def create_grid_map(self, x_size, y_size, resolution):
        self.x_size = x_size
        self.y_size = y_size
        self.resolution = resolution

        self.x_grid = int(self.x_size / resolution) + 2
        self.y_grid = int(self.y_size / resolution) + 2

        self.x_center = self.x_grid // 2
        self.y_center = self.y_grid // 2

        self.grid = np.zeros((self.x_grid, self.y_grid))

        self.prob_plus = 0.7
        self.prob_minus = 0.3

        self.grid_img = self.ax.imshow(
            self.grid.T,
            extent=[
                -self.x_size / 2,
                self.x_size / 2,
                -self.y_size / 2,
                self.y_size / 2,
            ],
            origin="lower",
            cmap="gray",
            vmin=0.0,
            vmax=1.0,
            alpha=0.5,
        )

    def update_map(self, x_values, y_values):
        x = np.array(x_values / self.resolution, dtype=np.int32) + self.x_center
        y = np.array(y_values / self.resolution, dtype=np.int32) + self.y_center

        valid_mask = (x >= 0) & (x < self.x_grid) & (y >= 0) & (y < self.y_grid)
        x = x[valid_mask]
        y = y[valid_mask]
        mask = np.ones(self.grid.shape, dtype=bool)
        mask[x, y] = False

        self.grid[mask] -= self.prob_minus
        self.grid[x, y] += self.prob_plus
        self.grid = np.clip(self.grid, 0.0, 1.0)


def main(args=None):
    rclpy.init(args=args)
    lidar_plot_ = LidarPlot()
    rclpy.spin(lidar_plot_)
    plt.close(all)
    lidar_plot_.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
