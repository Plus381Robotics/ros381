#include "../include/signal.hpp"
#include <example_interfaces/msg/int8.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ObstacleNode : public rclcpp::Node
{
  public:
    ObstacleNode() : Node("obstacle")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleNode::check_scan, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ObstacleNode::set_odom, this, std::placeholders::_1));

        obstacle_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("obstacle_status", 10);

        // obstacle_dir_sub_ = this->create_subscription<example_interfaces::msg::Int8>(
        // "obstacle_direction", 10, std::bind(&ObstacleNode::callback_obstacle_dir, this, std::placeholders::_1));

        // num_pts_part_ = resolution_ / 6.0;
        // double angle_increment_ = 2 * M_PI / resolution_;
        // for (int i = 0; i < 3200; i++)
        // {
        //     sin_lut_[i] = sin(i * angle_increment_);
        //     cos_lut_[i] = cos(i * angle_increment_);
        // }
    }

  private:
    bool lut_initialized_ = false;
    double v_base_, w_base_, v_eps_ = 0.05;
    double x_base_, y_base_, phi_base_;
    unsigned resolution_ = 3200, threshold_ = 40;    // TODO: u parametre
    double TABLE_X_LIMIT = 1.35, TABLE_Y_LIMIT = 0.85; // TODO: u parametre
    double y_max_, y_max_slow_, x_max_, x_max_slow_;
    double j_max_ = 40.0;                                         // TODO: parametar
    double robot_y_ = 0.16, robot_y_max_ = 0.25, dis_stop_ = 0.1; // TODO: parametri
    double robot_x = 0.18, robot_x_max = 0.25;
    double robot_y_slow_ = 0.1, dis_slow_ = 0.5; // TODO: parametri
    int num_pts_part_ = 533;
    // int num_offset_;
    int start_pt_ = 0, end_pt_ = 0;
    uint8_t obstacle_status_ = 0;
    int8_t obstacle_dir_ref_ = 0;
    uint8_t obstacle_dir_vel_ = 0;
    bool check_forw_ = false, check_back_ = false;
    double sin_lut_[3200], cos_lut_[3200];
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr obstacle_pub_;
    rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr obstacle_dir_sub_;

    void callback_obstacle_dir(const example_interfaces::msg::Int8::SharedPtr msg)
    {
        // obstacle_dir_ref_ = msg->data;
    }

    void set_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        v_base_ = msg->twist.twist.linear.x;
        w_base_ = msg->twist.twist.angular.z;
        x_base_ = msg->pose.pose.position.x;
        y_base_ = msg->pose.pose.position.y;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        phi_base_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        // if (v_base_ < 0.0)
        //     num_offset_ = num_pts_part_;
        // else
        //     num_offset_ = 0;
    }

    void check_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!lut_initialized_)
        {
            resolution_ = msg->ranges.size();

            num_pts_part_ = resolution_ / 6;

            for (int i = 0; i < resolution_; i++)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                sin_lut_[i] = sin(angle);
                cos_lut_[i] = cos(angle);
            }

            lut_initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "LUT initialized: res=%d angle_min=%.3f inc=%.6f", resolution_,
                        msg->angle_min, msg->angle_increment);
        }
        if (fabs(v_base_) < v_eps_)
            obstacle_dir_ref_ = 0;
        else
            obstacle_dir_ref_ = get_sign(v_base_);
        obstacle_status_ = 0;
        unsigned stop_num = 0;
        unsigned slow_num = 0;

        x_max_ = 5.0 / 3.0 * pow(fabs(v_base_ * 2.0), 5.0 / 3.0) / sqrt(j_max_) + robot_x + robot_x_max + dis_stop_;
        x_max_slow_ = x_max_ + dis_slow_;
        y_max_ = robot_y_ + robot_y_max_;
        y_max_slow_ = y_max_ + robot_y_slow_;
        if (obstacle_dir_ref_ != 0)
        {

            if (obstacle_dir_ref_ == 1)
            {
                start_pt_ = -num_pts_part_;
                end_pt_ = num_pts_part_;
            }
            else
            {
                start_pt_ = resolution_ / 2 - num_pts_part_;
                end_pt_ = resolution_ / 2 + num_pts_part_;
            }

            for (int i = start_pt_; i <= end_pt_; i++)
            {
                unsigned ui = (i + resolution_) % resolution_;
                double range = msg->ranges[ui];
                if (!std::isfinite(range))
                    continue;
                if (range >= robot_x && range <= 1.75)
                {
                    double obst_x_robot = range * cos_lut_[ui];
                    double obst_y_robot = range * sin_lut_[ui];

                    double c = cos(phi_base_);
                    double s = sin(phi_base_);

                    double obst_x_table = x_base_ + obst_x_robot * cos(phi_base_) + obst_y_robot * sin(phi_base_);
                    double obst_y_table = y_base_ - obst_x_robot * sin(phi_base_) + obst_y_robot * cos(phi_base_);

                    // RCLCPP_INFO(this->get_logger(), "Point table: [ %.2f, %.2f]", obst_x_table, obst_y_table);

                    if (fabs(obst_x_table) > TABLE_X_LIMIT || fabs(obst_y_table) > TABLE_Y_LIMIT || obst_y_table > 0.6)
                        continue;
                    if (fabs(obst_y_robot) > y_max_slow_ || fabs(obst_x_robot) > x_max_slow_ + dis_slow_)
                        continue;
                    if (fabs(obst_y_robot) > y_max_ || fabs(obst_x_robot) > x_max_ + dis_stop_)
                    {
                        slow_num++;
                        continue;
                    }
                    stop_num++;
                }
            }
            if (stop_num >= threshold_)
                obstacle_status_ = 1;
            else if (slow_num + stop_num >= threshold_)
                obstacle_status_ = 2;
        }
        else
        {
            for (int i = 0; i < resolution_; i++)
            {
                unsigned ui = i;
                double range = msg->ranges[ui];
                if (!std::isfinite(range))
                    continue;
                if (range >= robot_x && range <= 1.75)
                {
                    double obst_x_robot = range * cos_lut_[ui];
                    double obst_y_robot = range * sin_lut_[ui];
                    
                    double c = cos(phi_base_);
                    double s = sin(phi_base_);

                    double obst_x_table = x_base_ + obst_x_robot * cos(phi_base_) + obst_y_robot * sin(phi_base_);
                    double obst_y_table = y_base_ - obst_x_robot * sin(phi_base_) + obst_y_robot * cos(phi_base_);

                    // RCLCPP_INFO(this->get_logger(), "Point table: [ %.2f, %.2f], robot [ %.2f, %.2f]", obst_x_table, obst_y_table, obst_x_robot, obst_y_robot);

                    if (fabs(obst_x_table) > TABLE_X_LIMIT || fabs(obst_y_table) > TABLE_Y_LIMIT)
                        continue;

                    if (fabs(obst_y_robot) > y_max_slow_ || fabs(obst_x_robot) > x_max_slow_ + dis_slow_)
                        continue;

                    if (fabs(obst_y_robot) > y_max_ || fabs(obst_x_robot) > x_max_ + dis_stop_)
                    {
                        slow_num++;
                        continue;
                    }

                    stop_num++;
                }
            }

            if (stop_num >= threshold_ * 2)
                obstacle_status_ = 1;
        }
        publish_obstacle();
        // RCLCPP_INFO(this->get_logger(), "Obstacle: direction = %d, status = %d, stop_num = %d", obstacle_dir_ref_,
                    // obstacle_status_, stop_num);
    }

    void publish_obstacle()
    {
        auto obst_msg = example_interfaces::msg::UInt8();
        obst_msg.data = obstacle_status_;
        obstacle_pub_->publish(obst_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
