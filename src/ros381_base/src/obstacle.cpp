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

        num_pts_half_ = resolution_ / 4;
    }

  private:
    double v_base_, w_base_;
    unsigned resolution_ = 3200, threshold_ = 5; // TODO: u parametre
    double y_max_, y_max_slow_, x_max_, x_max_slow_;
    double j_max_ = 50.0;                                                           // TODO: parametar
    double inf_x_stop_ = 0.1, robot_l_ = 0.15, inf_y_stop_ = 0.22, dis_stop_ = 0.2; // TODO: parametri
    double inf_y_slow_ = 0.05, dis_slow_ = 1.0;                                     // TODO: parametri
    int num_pts_half_ = 800, num_offset_;
    uint8_t obstacle_status_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr obstacle_pub_;

    void set_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        v_base_ = msg->twist.twist.linear.x;
        w_base_ = msg->twist.twist.angular.z;
    }

    void check_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        unsigned stop_num = 0;
        unsigned slow_num = 0;
        x_max_ = 5 / 3 * pow(fabs(v_base_), 5 / 3) / sqrt(j_max_) + inf_y_stop_ + inf_x_stop_ + dis_stop_;
        x_max_slow_ = x_max_ + dis_slow_;
        y_max_ = robot_l_ + inf_y_stop_;
        y_max_slow_ = y_max_ + inf_y_slow_;
        // RCLCPP_INFO(this->get_logger(), "\n\n\n");
        // if (v_base_ < 0.0)
        // num_offset_ = num_pts_half_;
        // else
        num_offset_ = 0;
        for (int i = num_offset_ - num_pts_half_; i <= num_pts_half_ + num_offset_; i++)
        {
            unsigned ui = (i + resolution_) % resolution_;
            double range = msg->ranges[ui];
            // RCLCPP_INFO(this->get_logger(), "\ni = %d , ui =%d", i, ui);
            // RCLCPP_INFO(this->get_logger(), "\ndistance = %.2f m, angle =%.2f degrees", range,
            //             ui * msg->angle_increment * 180 / M_PI);
            if (range >= msg->range_min && range <= msg->range_max)
            {
                double angle = ui * msg->angle_increment;
                double r_x = fabs(range * cos(angle));
                double r_y = fabs(range * sin(angle));
                // RCLCPP_INFO(this->get_logger(), "point@%.2f degrees = (%.2f, %.2f)", angle * 180 / M_PI, r_x, r_y);
                if (r_y < y_max_ && r_x < x_max_)
                {
                    stop_num++;
                    // RCLCPP_INFO(this->get_logger(), "point@%.2f degrees = (%.2f, %.2f)", angle * 180 / M_PI, r_x,
                    // r_y);
                }
                else if (r_y < y_max_slow_ && r_x < x_max_slow_)
                    slow_num++;
            }
        }
        if (stop_num >= threshold_)
            obstacle_status_ = 1;
        else if (slow_num + stop_num >= threshold_)
            obstacle_status_ = 2;
        else
            obstacle_status_ = 0;
        publish_obstacle();
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
