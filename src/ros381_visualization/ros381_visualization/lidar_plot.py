import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from ros381_interfaces.msg import Float3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow


class GridmapPlot(Node):
    def __init__(self):
        super().__init__("gridmap_plot")
        self.declare_parameter("robot_name", "ros381")
        self.robot_ = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )

        # subs
        self.gridmap_sub_ = self.create_subscription(
            OccupancyGrid, f"/{self.robot_}/gridmap", self.plot_gridmap, 10
        )
        self.odom_sub_ = self.create_subscription(
            Odometry, f"/{self.robot_}/odom", self.set_odom, 10
        )
        self.pose_offset_sub_ = self.create_subscription(
            Float3, f"/{self.robot_}/pose_offset", self.offset_pose, 1
        )

        # state
        self.robot_x_ = 0.0
        self.robot_y_ = 0.0
        self.robot_phi_ = 0.0
        self.odom_set_ = False
        self.pose_set_ = False

        # plotting setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.0, 1.0)
        self.ax.grid(True)
        self.robot_arrow = None
        self.grid_img = None
        plt.tight_layout()
        plt.show(block=False)

        self.get_logger().info("GridmapPlot node started.")

    def offset_pose(self, msg: Float3):
        self.robot_x_ += msg.float3[0]
        self.robot_y_ += msg.float3[1]
        self.robot_phi_ += msg.float3[2]
        self.pose_set_ = True

    def set_odom(self, msg: Odometry):
        self.odom_set_ = True
        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.robot_phi_ = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.robot_x_ = msg.pose.pose.position.x
        self.robot_y_ = msg.pose.pose.position.y

    def plot_gridmap(self, msg: OccupancyGrid):
        if not self.odom_set_:
            return

        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        # grid = grid.T  # transpose so x=width, y=height
        extent = [
            msg.info.origin.position.x,
            msg.info.origin.position.x + msg.info.width * msg.info.resolution,
            msg.info.origin.position.y,
            msg.info.origin.position.y + msg.info.height * msg.info.resolution,
        ]

        if self.grid_img is None:
            self.grid_img = self.ax.imshow(
                grid,
                extent=extent,
                origin="lower",
                cmap="gray",
                vmin=0,
                vmax=100,
                alpha=0.6,
            )
        else:
            self.grid_img.set_data(grid)
            self.grid_img.set_extent(extent)

        if self.pose_set_:
            if self.robot_arrow is not None:
                self.robot_arrow.remove()
            arrow_length = 0.2
            dx = arrow_length * np.cos(self.robot_phi_)
            dy = arrow_length * np.sin(self.robot_phi_)
            self.robot_arrow = Arrow(
                self.robot_x_, self.robot_y_, dx, dy, width=0.1, color="red"
            )
            self.ax.add_patch(self.robot_arrow)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = GridmapPlot()
    rclpy.spin(node)
    plt.close("all")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
