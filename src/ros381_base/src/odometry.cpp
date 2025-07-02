#include "nav_msgs/msg/odometry.hpp"
#include "../include/signal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "tf2/LinearMath/Quaternion.h"

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode () : Node ("odometry")
  {
    this->declare_parameter ("d_right", 0.072);
    r_right_ = this->get_parameter ("d_right").as_double () * 0.5f;
    this->declare_parameter ("d_left", 0.072);
    r_left_ = this->get_parameter ("d_left").as_double () * 0.5f;
    this->declare_parameter ("L", 0.297);
    L_ = this->get_parameter ("L").as_double ();
    L_recip_ = 1 / L_;

    this->set_parameter (rclcpp::Parameter ("use_sim_time", true));

    passive_vel_sub_
        = this->create_subscription<ros381_interfaces::msg::Float2> (
            "base_encoders", 10,
            std::bind (&OdometryNode::callbackPassiveVel, this,
                       std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry> ("odom", 10);

    clock_ = this->get_clock ();

    RCLCPP_INFO (this->get_logger (), "Odometry node is running.");
  }

private:
  void
  callbackPassiveVel (const ros381_interfaces::msg::Float2::SharedPtr msg)
  {
    delta_phi_wheels_ = *msg;
    calculate_odometry ();
  }

  void
  calculate_odometry ()
  {
    if (odom_initialized_)
      {
        current_time_ = clock_->now ();
        dt_ = (current_time_ - previous_time_).seconds ();
        // RCLCPP_INFO (this->get_logger(), "dt_ = %f", dt_);

        w_wheels_.float2[0] = delta_phi_wheels_.float2[0] / dt_; // [rad/s]
        w_wheels_.float2[1] = delta_phi_wheels_.float2[1] / dt_; // [rad/s]
        v_wheels_.float2[0] = w_wheels_.float2[0] * r_right_;    // [m/s]
        v_wheels_.float2[1] = w_wheels_.float2[1] * r_left_;     // [m/s]
        // RCLCPP_INFO (this->get_logger (), "Got passive vel! (%.4f, %.4f)",
        //              v.float2[0], v.float2[1]);

        v_base_ = (v_wheels_.float2[0] + v_wheels_.float2[1]) * 0.5;
        w_base_ = (v_wheels_.float2[0] - v_wheels_.float2[1]) * L_recip_;
        mid_angle_ = phi_base_ + w_base_ * dt_ * 0.5;

        x_base_ += v_base_ * cos (mid_angle_) * dt_;
        y_base_ += v_base_ * sin (mid_angle_) * dt_;
        phi_base_ += w_base_ * dt_;
        wrapPi_ptr (&phi_base_);

        previous_time_ = current_time_;
        RCLCPP_INFO (this->get_logger (),
                     "\nv = %.6f\nw = %.6f\nx = %.6f\ny = %.6f\nphi = %.6f",
                     v_base_, w_base_, x_base_, y_base_, phi_base_);
        this->publish_odometry ();
      }
    else
      {
        odom_initialized_ = true;
        previous_time_ = clock_->now ();
        RCLCPP_INFO (this->get_logger (), "Odometry initialized!");
      }
  }

  void
  publish_odometry ()
  {
    auto msg = nav_msgs::msg::Odometry ();
    // Set the header
    msg.header.stamp = current_time_;
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

  double r_right_;                                  // [m]
  double r_left_;                                   // [m]
  double L_;                                        // [m]
  double L_recip_;                                  // [1/m]
  double rad2deg = 180 / M_PI;                      // [deg/rad]
  double deg2rad = M_PI / 180;                      // [rad/deg]
  double dt_;                                       // [s]
  double v_base_;                                   // [m/s]
  double w_base_;                                   // [rad/s]
  double phi_base_ = 0;                             // [rad]
  double mid_angle_;                                // [rad]
  double x_base_ = 0;                               // [m]
  double y_base_ = 0;                               // [m]
  ros381_interfaces::msg::Float2 delta_phi_wheels_; // [rad]
  ros381_interfaces::msg::Float2 w_wheels_;         // [rad/s]
  ros381_interfaces::msg::Float2 v_wheels_;         // [m/s]
  bool odom_initialized_ = false;

  rclcpp::Time current_time_, previous_time_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<ros381_interfaces::msg::Float2>::SharedPtr
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
