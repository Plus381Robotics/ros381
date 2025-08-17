#include <algorithm>
#include <cmath>
#include <deque>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros381_interfaces/msg/float3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

struct OdomEntry
{
    double t;
    double x;
    double y;
    double phi;
};

class Gridmap : public rclcpp::Node
{
  public:
    Gridmap() : Node("gridmap")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Gridmap::update_map, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Gridmap::set_odom, this, std::placeholders::_1));

        pose_offset_sub_ = this->create_subscription<ros381_interfaces::msg::Float3>(
            "pose_offset", 1, std::bind(&Gridmap::offset_pose, this, std::placeholders::_1));

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 10);

        create_grid_map(3.0, 2.0, 0.05);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ros381_interfaces::msg::Float3>::SharedPtr pose_offset_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    double robot_x_ = 0.0, robot_y_ = 0.0, robot_phi_ = 0.0;
    bool odom_set_ = false, pose_set_ = false;
    std::deque<OdomEntry> odom_buffer_;

    double x_size_, y_size_, resolution_;
    int x_grid_, y_grid_;
    int x_center_, y_center_;
    double prob_plus_, prob_minus_;
    std::vector<std::vector<double>> grid_;

    void offset_pose(const ros381_interfaces::msg::Float3::SharedPtr msg)
    {
        robot_x_ += msg->float3[0];
        robot_y_ += msg->float3[1];
        robot_phi_ += msg->float3[2];
        pose_set_ = true;
    }

    void set_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_set_ = true;
        double t_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double phi = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        odom_buffer_.push_back({t_stamp, x, y, phi});
    }

    std::tuple<double, double, double> interp_pose(double t_req)
    {
        if (odom_buffer_.empty())
            return {robot_x_, robot_y_, robot_phi_};

        for (size_t i = 1; i < odom_buffer_.size(); ++i)
        {
            if (t_req < odom_buffer_[i].t)
            {
                const auto &o1 = odom_buffer_[i - 1];
                const auto &o2 = odom_buffer_[i];
                double ratio = (t_req - o1.t) / (o2.t - o1.t);
                double x = o1.x + ratio * (o2.x - o1.x);
                double y = o1.y + ratio * (o2.y - o1.y);
                double phi = o1.phi + ratio * (o2.phi - o1.phi);
                return {x, y, phi};
            }
        }

        const auto &last = odom_buffer_.back();
        return {last.x, last.y, last.phi};
    }

    void create_grid_map(double x_size, double y_size, double resolution)
    {
        x_size_ = x_size;
        y_size_ = y_size;
        resolution_ = resolution;
        x_grid_ = static_cast<int>(x_size_ / resolution_) + 2;
        y_grid_ = static_cast<int>(y_size_ / resolution_) + 2;
        x_center_ = x_grid_ / 2;
        y_center_ = y_grid_ / 2;
        grid_ = std::vector<std::vector<double>>(x_grid_, std::vector<double>(y_grid_, 0.0));
        prob_plus_ = 0.6;
        prob_minus_ = 0.1;
    }

    void update_map(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_set_ || odom_buffer_.empty())
            return;

        nav_msgs::msg::OccupancyGrid msg_grid;
        msg_grid.header.stamp = this->now();
        msg_grid.header.frame_id = "odom";
        msg_grid.info.width = x_grid_;
        msg_grid.info.height = y_grid_;
        msg_grid.info.resolution = resolution_;
        msg_grid.info.origin.position.x = -x_size_ / 2.0;
        msg_grid.info.origin.position.y = -y_size_ / 2.0;
        msg_grid.info.origin.orientation.w = 1.0;
        msg_grid.data.resize(x_grid_ * y_grid_);
        msg_grid.data.resize(x_grid_ * y_grid_);

        robot_x_ = odom_buffer_.back().x;
        robot_y_ = odom_buffer_.back().y;
        robot_phi_ = odom_buffer_.back().phi;

        double t_end = this->now().nanoseconds() * 1e-9;
        double scan_duration = msg->scan_time;

        std::vector<double> x_world, y_world;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            auto [rx, ry, rphi] = interp_pose(t_end - (scan_duration - i * msg->time_increment));
            x_world.push_back(
                rx +
                msg->ranges[i] *
                    std::cos(msg->angle_min + i * (msg->angle_max - msg->angle_min) / (msg->ranges.size() - 1) - rphi));
            y_world.push_back(
                ry -
                msg->ranges[i] *
                    std::sin(msg->angle_min + i * (msg->angle_max - msg->angle_min) / (msg->ranges.size() - 1) - rphi));
        }

        auto last = odom_buffer_.back();
        odom_buffer_.clear();
        odom_buffer_.push_back(last);

        std::vector<int> valid_x, valid_y;
        for (size_t i = 0; i < x_world.size(); ++i)
        {
            int xi = static_cast<int>(x_world[i] / resolution_) + x_center_;
            int yi = static_cast<int>(y_world[i] / resolution_) + y_center_;
            if (xi >= 0 && xi < x_grid_ && yi >= 0 && yi < y_grid_)
            {
                valid_x.push_back(xi);
                valid_y.push_back(yi);
            }
        }
        for (int x = 0; x < x_grid_; ++x)
        {
            for (int y = 0; y < y_grid_; ++y)
            {
                grid_[x][y] -= prob_minus_;

                for (size_t k = 0; k < valid_x.size(); ++k)
                {
                    if (valid_x[k] == x && valid_y[k] == y)
                    {
                        grid_[x][y] += prob_plus_;
                        break;
                    }
                }

                grid_[x][y] = std::clamp(grid_[x][y], 0.0, 1.0);

                msg_grid.data[y * x_grid_ + x] = static_cast<int>(grid_[x][y] * 100);
            }
        }

        grid_pub_->publish(msg_grid);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Gridmap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}