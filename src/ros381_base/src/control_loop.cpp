#include "../include/signal.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode () : Node ("control_loop")
  {
    this->declare_parameter ("freq_hz", 10.0);
    freq_hz_ = this->get_parameter ("freq_hz").as_double ();
    period_us_ = 1000000 / freq_hz_;

    // TODO:
    // - uzima mete: action
    timer_ = this->create_wall_timer (
        std::chrono::microseconds (period_us_),
        std::bind (&ControlLoopNode::control_loop, this));
    motor_cmd_publisher_
        = this->create_publisher<ros381_interfaces::msg::Float2> ("motor_cmd",
                                                                  10);
    odometry_subscription_
        = this->create_subscription<nav_msgs::msg::Odometry> (
            "odom", 10,
            std::bind (&ControlLoopNode::callback_odometry, this,
                       std::placeholders::_1));

    RCLCPP_INFO (this->get_logger (), "Control loop node is running.");
  }

private:
  double phi_base_;                                          // [rad]
  double x_base_, y_base_;                                   // [m]
  double v_base_, V_MAX_ = 2.0, V_MIN_ = 0.05, v_ref_ = 0.0; // [m/s]
  double w_base_, W_MAX_ = 12.6, W_MIN_ = 0.3, w_ref_ = 0.0; // [rad/s]
  double x_ref_ = 0.0, y_ref_ = 0.0, phi_ref_ = 1.57;        // [m]
  double x_error_, y_error_;                                 // [m]
  double phi_error_;                                         // [rad]
  double distance_;                                          // [m]
  double distance_proj_;                                     //[m]
  double EPS_ = 0.01;
  double freq_hz_;
  int64_t period_us_;
  double v_right = 0.0, v_left = 0.0; // [m/s]
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros381_interfaces::msg::Float2>::SharedPtr
      motor_cmd_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;

  void
  control_loop ()
  {
    // input:         reference positions     (x_ref, y_ref [m], phi_ref [rad])
    // TODO: ovde ide mali fsm: rotate, go_to_xy, 0
    x_error_ = x_ref_ - x_base_;
    y_error_ = y_ref_ - y_base_;
    phi_error_ = std::clamp (phi_ref_ - phi_base_, -M_PI, M_PI);
    // RCLCPP_INFO (this->get_logger (), "phi_error = %.5f", phi_error_);

    distance_ = sqrt (x_error_ * x_error_ + y_error_ * y_error_);
    // RCLCPP_INFO(this->get_logger(), "v_ref_ = %.2f, w_ref_ = %.2f", v_ref_,
    // w_ref_);

    // intermediate:  reference velocities    (v_ref [m/s], w_ref [rad/s])
    v_ref_ = synthesis_7 (distance_, v_base_, 0.25, V_MAX_, V_MIN_,
    0.5, 2.5);
    w_ref_ = synthesis_7 (phi_error_, w_base_, 0.77, W_MAX_, W_MIN_, 0.5, 2.5);
    // RCLCPP_INFO (this->get_logger (), "w_ref_ = %.2f, w_base_ = %.2f",
    // w_ref_, w_base_);

    // output:      reference motor commands  (v_right, v_left) [m/s]
    v_right = v_ref_ + w_ref_;
    v_left = v_ref_ - w_ref_;
    RCLCPP_INFO (this->get_logger (), "v_right = %.2f, v_left = %.2f", v_right,
                 v_left);
    this->publish_motor_cmd ();
  }

  void
  publish_motor_cmd ()
  {
    auto msg = ros381_interfaces::msg::Float2 ();
    msg.float2[0] = v_right;
    msg.float2[1] = v_left;
    motor_cmd_publisher_->publish (msg);
  }

  void
  callback_odometry (const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_base_ = msg->pose.pose.position.x;
    y_base_ = msg->pose.pose.position.y;
    phi_base_ = tf2::getYaw (msg->pose.pose.orientation);
    v_base_ = msg->twist.twist.linear.x;
    w_base_ = msg->twist.twist.angular.z;
    // RCLCPP_INFO (this->get_logger (),
    //              "\nv = %.3f\nw = %.3f\nx = %.3f\ny = %.3f\nphi = %.3f",
    //              v_base_, w_base_, x_base_, y_base_, phi_base_);
  }

  double
  synthesis_7 (double distance, double velocity, double stopping_distance,
               double V_MAX, double V_MIN, double velocity_percentage = 1.0,
               double k_acc = 2.5)
  {
    double v_max_ = V_MAX * std::min (1.0, std::fabs (velocity_percentage));
    double v_ref_ = 0;
    double x = 0;

    if (distance <= stopping_distance * (1.0f + EPS_))
      {
        distance = std::clamp (distance, 0.0, stopping_distance);
        x = distance / stopping_distance; // Decel phase
      }
    else
      {
        velocity = std::clamp (velocity, V_MIN, v_max_);
        x = pow (velocity / v_max_, 1.0 / k_acc); // Accel phase (tunable)
        // RCLCPP_INFO (this->get_logger (), "x = %.2f", x);
      }
    x = std::clamp (x, 0.0, 1.0);
    v_ref_ = v_max_
             * (35.0f * pow (x, 4) - 84.0f * pow (x, 5) + 70.0f * pow (x, 6)
                - 20.0f * pow (x, 7));
    return v_ref_;
  }
};

int
main (int argc, char **argv)
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<ControlLoopNode> ();
  sleep (1);
  rclcpp::spin (node);
  rclcpp::shutdown ();
  return 0;
}
