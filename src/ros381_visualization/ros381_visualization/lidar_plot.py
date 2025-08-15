import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np


class LidarPlot(Node):
    def __init__(self):
        super().__init__("lidar_plot")
        self.declare_parameter("robot_name", "ros381")
        self.robot_ = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.subscription_ = self.create_subscription(
            LaserScan, f"/{self.robot_}/scan", self.plot_scan, 10
        )

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [], s=5)
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-2, 2)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("LIDAR Scan Data (XY Coordinates)")
        self.ax.grid(True)

        # Draw robot position (center)
        self.ax.plot(0, 0, "ro", markersize=5)
        self.ax.text(0, 0.1, "Robot", ha="center")

        # Draw arena boundaries (rectangle)
        arena_width = 3
        arena_height = 2
        self.ax.plot(
            [
                -arena_width / 2,
                arena_width / 2,
                arena_width / 2,
                -arena_width / 2,
                -arena_width / 2,
            ],
            [
                arena_height / 2,
                arena_height / 2,
                -arena_height / 2,
                -arena_height / 2,
                arena_height / 2,
            ],
            "r--",
            linewidth=1,
        )
        plt.tight_layout()
        plt.show(block=False)
        self.once_ = True

    def plot_scan(self, msg):
        angles = np.arange(
            msg.angle_min, msg.angle_max + msg.angle_increment / 2, msg.angle_increment
        )
        angles = angles[: len(msg.ranges)]
        ranges = np.array(msg.ranges)

        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = angles[valid_mask]
        ranges = ranges[valid_mask]
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        self.scatter.set_offsets(np.column_stack((x, y)))
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
