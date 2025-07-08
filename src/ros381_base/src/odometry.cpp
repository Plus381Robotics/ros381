#include "nav_msgs/msg/odometry.hpp"
#include "../include/signal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros381_interfaces/msg/float3.hpp"
#include "tf2/LinearMath/Quaternion.h"

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode () : Node ("odometry")
  {
    this->declare_parameter ("L", 0.297);
    L_ = this->get_parameter ("L").as_double ();
    L_recip_ = 1 / L_;
    this->set_parameter (rclcpp::Parameter ("use_sim_time", true));

    passive_vel_sub_
        = this->create_subscription<ros381_interfaces::msg::Float3> (
            "base_encoders", 10,
            std::bind (&OdometryNode::callback_passive_vel, this,
                       std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry> ("odom", 10);

    RCLCPP_INFO (this->get_logger (), "Odometry node is running.");
  }

private:
  void
  callback_passive_vel (const ros381_interfaces::msg::Float3::SharedPtr msg)
  {
    v_right_ = msg->float3[0];
    v_left_ = msg->float3[1];
    dt_ = msg->float3[2];
    calculate_odometry ();
  }

  void
  calculate_odometry ()
  {
    if (odom_initialized_)
      {
        // RCLCPP_INFO (this->get_logger(), "dt_ = %f", dt_);

        // RCLCPP_INFO (this->get_logger (), "Got passive vel! (%.4f, %.4f)",
        //              v.float2[0], v.float2[1]);

        v_base_ = (v_right_ + v_left_) * 0.5;
        w_base_ = (v_right_ - v_left_) * L_recip_;
        mid_angle_ = phi_base_ + w_base_ * dt_ * 0.5;

        x_base_ += v_base_ * cos (mid_angle_) * dt_;
        y_base_ += v_base_ * sin (mid_angle_) * dt_;
        phi_base_ += w_base_ * dt_;
        wrap_ptr (&phi_base_, M_PI, -M_PI);

        // RCLCPP_INFO (this->get_logger (),
        //              "\nv = %.3f\nw = %.3f\nx = %.3f\ny = %.3f\nphi = %.3f",
        //              v_base_, w_base_, x_base_, y_base_, phi_base_);
        this->publish_odometry ();
      }
    else
      {
        odom_initialized_ = true;
        RCLCPP_INFO (this->get_logger (), "Odometry initialized!");
      }
  }

  void
  publish_odometry ()
  {
    auto msg = nav_msgs::msg::Odometry ();
    // Set the header
    msg.header.stamp = now ();
    msg.header.frame_id = "odom";     // Adjust as needed
    msg.child_frame_id = "base_link"; // Adjust as needed

    // Set position
    msg.pose.pose.position.x = x_base_;
    msg.pose.pose.position.y = y_base_;
    msg.pose.pose.position.z = 0.0;

    // Convert yaw (phi_base_) to quaternion
    tf2::Quaternion q;
    q.setRPY (0, 0, phi_base_);
    msg.pose.pose.orientation.x = q.x ();
    msg.pose.pose.orientation.y = q.y ();
    msg.pose.pose.orientation.z = q.z ();
    msg.pose.pose.orientation.w = q.w ();

    // Set linear and angular velocity
    msg.twist.twist.linear.x = v_base_;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;

    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = w_base_;
    odom_pub_->publish (msg);
  }

  double L_;            // [m]
  double L_recip_;      // [1/m]
  double v_right_;      // [m/s]
  double v_left_;       // [m/s]
  double dt_;           // [s]
  double v_base_;       // [m/s]
  double w_base_;       // [rad/s]
  double phi_base_ = 0; // [rad]
  double mid_angle_;    // [rad]
  double x_base_ = 0;   // [m]
  double y_base_ = 0;   // [m]
  bool odom_initialized_ = false;

  rclcpp::Subscription<ros381_interfaces::msg::Float3>::SharedPtr
      passive_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int
main (int argc, char **argv)
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<OdometryNode> ();
  sleep (1);
  rclcpp::spin (node);
  rclcpp::shutdown ();
  return 0;
}
