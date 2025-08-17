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
    Gridmap() : Node("gridmap"), robot_x_(0.0), robot_y_(0.0), robot_phi_(0.0), odom_set_(false), pose_set_(false)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Gridmap::plot_scan, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Gridmap::set_odom, this, std::placeholders::_1));

        pose_offset_sub_ = this->create_subscription<ros381_interfaces::msg::Float3>(
            "pose_offset", 1, std::bind(&Gridmap::offset_pose, this, std::placeholders::_1));

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 10);

        create_grid_map(3.0, 2.0, 0.05);
    }

  private:
    // subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ros381_interfaces::msg::Float3>::SharedPtr pose_offset_sub_;

    // publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    // state
    std::string robot_;
    double robot_x_, robot_y_, robot_phi_;
    bool odom_set_;
    bool pose_set_;
    std::deque<OdomEntry> odom_buffer_;

    // grid map
    double x_size_, y_size_, resolution_;
    int x_grid_, y_grid_;
    int x_center_, y_center_;
    double prob_plus_, prob_minus_;
    std::vector<std::vector<double>> grid_;

    // --------------------------- methods ------------------------------

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

        std::vector<double> times, xs, ys, phis;
        for (const auto &o : odom_buffer_)
        {
            times.push_back(o.t);
            xs.push_back(o.x);
            ys.push_back(o.y);
            phis.push_back(o.phi);
        }

        // unwrap phi
        for (size_t i = 1; i < phis.size(); ++i)
        {
            while (phis[i] - phis[i - 1] > M_PI)
                phis[i] -= 2 * M_PI;
            while (phis[i] - phis[i - 1] < -M_PI)
                phis[i] += 2 * M_PI;
        }

        auto interp = [&](const std::vector<double> &v) {
            if (t_req <= times.front())
                return v.front();
            if (t_req >= times.back())
                return v.back();
            for (size_t i = 1; i < times.size(); ++i)
            {
                if (t_req < times[i])
                {
                    double ratio = (t_req - times[i - 1]) / (times[i] - times[i - 1]);
                    return v[i - 1] + ratio * (v[i] - v[i - 1]);
                }
            }
            return v.back();
        };

        double x = interp(xs);
        double y = interp(ys);
        double phi = interp(phis);
        return {x, y, phi};
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

    void update_map(const std::vector<double> &x_values, const std::vector<double> &y_values)
    {
        std::vector<int> valid_x, valid_y;
        for (size_t i = 0; i < x_values.size(); ++i)
        {
            int xi = static_cast<int>(x_values[i] / resolution_) + x_center_;
            int yi = static_cast<int>(y_values[i] / resolution_) + y_center_;
            if (xi >= 0 && xi < x_grid_ && yi >= 0 && yi < y_grid_)
            {
                valid_x.push_back(xi);
                valid_y.push_back(yi);
            }
        }

        // mask logic same as Python
        for (int i = 0; i < x_grid_; ++i)
            for (int j = 0; j < y_grid_; ++j)
                grid_[i][j] -= prob_minus_;
        for (size_t i = 0; i < valid_x.size(); ++i)
            grid_[valid_x[i]][valid_y[i]] += prob_plus_;
        for (int i = 0; i < x_grid_; ++i)
            for (int j = 0; j < y_grid_; ++j)
                grid_[i][j] = std::clamp(grid_[i][j], 0.0, 1.0);

        publish_gridmap();
    }

    void plot_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_set_ || odom_buffer_.empty())
            return;

        robot_x_ = odom_buffer_.back().x;
        robot_y_ = odom_buffer_.back().y;
        robot_phi_ = odom_buffer_.back().phi;

        double t_end = this->now().nanoseconds() * 1e-9;
        double scan_duration = msg->scan_time;

        // remove old odom entries
        while (!odom_buffer_.empty() && odom_buffer_.front().t < t_end - scan_duration)
            odom_buffer_.pop_front();

        // compute beam times
        size_t n_beams = msg->ranges.size();
        double dt = (msg->time_increment > 0) ? msg->time_increment : scan_duration / std::max<int>(n_beams - 1, 1);
        std::vector<double> beam_times(n_beams);
        for (size_t i = 0; i < n_beams; ++i)
            beam_times[i] = t_end - (scan_duration - i * dt);

        // valid ranges
        std::vector<double> angles, ranges;
        for (size_t i = 0; i < n_beams; ++i)
        {
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
            {
                angles.push_back(msg->angle_min + i * (msg->angle_max - msg->angle_min) / (n_beams - 1));
                ranges.push_back(msg->ranges[i]);
            }
        }

        std::vector<double> x_world, y_world;
        for (size_t i = 0; i < ranges.size(); ++i)
        {
            auto [rx, ry, rphi] = interp_pose(beam_times[i]);
            x_world.push_back(rx + ranges[i] * std::cos(angles[i] - rphi));
            y_world.push_back(ry - ranges[i] * std::sin(angles[i] - rphi)); // exactly like Python
        }

        update_map(x_world, y_world);
        publish_gridmap();
    }

    void publish_gridmap()
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = robot_;
        msg.info.width = x_grid_;
        msg.info.height = y_grid_;
        msg.info.resolution = resolution_;
        msg.info.origin.position.x = -x_size_ / 2.0;
        msg.info.origin.position.y = -y_size_ / 2.0;
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(x_grid_ * y_grid_);
        for (int y = 0; y < y_grid_; ++y)
        {
            for (int x = 0; x < x_grid_; ++x)
            {
                msg.data[y * x_grid_ + x] = static_cast<int>(grid_[x][y] * 100);
            }
        }
        grid_pub_->publish(msg);
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