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
    this->declare_parameter ("freq_hz", 25.0);
    freq_ = this->get_parameter ("freq_hz").as_double ();
    period_us_ = 1000000 / freq_;

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
  double phi_base_;                                       // [rad]
  double x_base_, y_base_;                                // [m]
  double v_base_, V_MAX_ = 2.0, v_ref_ = 0.0, prev_v_;    // [m/s]
  double w_base_, W_MAX_ = 12.6, w_ref_ = 0.0, prev_w_;   // [rad/s]
  double x_ref_ = 1.0, y_ref_ = 0.0, phi_ref_ = 0 * 1.57; // [m]
  double x_error_, y_error_;                              // [m]
  double phi_error_;                                      // [rad]
  double distance_;                                       // [m]
  double distance_proj_;                                  //[m]
  double stopping_distance_ = 0.5;
  double stopping_angle_ = 0.77;
  double a_ /* [m/s^2] */, alpha_ /* [rad/s^2] */;
  double j_max_ = 20.0;               // [m/s^3]
  double j_rot_max = 120.0;           // [rad/s^3]
  unsigned long time_ns_, prev_time_; // [ns]
  double dt_;                         // [s]
  double freq_;                       // [Hz]
  int64_t period_us_;
  double v_right = 0.0, v_left = 0.0; // [m/s]
  bool odom_initialized_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros381_interfaces::msg::Float2>::SharedPtr
      motor_cmd_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;

  void
  control_loop ()
  {
    if (odom_initialized_)
      {
        // input:         reference positions     (x_ref, y_ref [m], phi_ref
        // [rad])
        // TODO: ovde ide mali fsm: rotate, go_to_xy, 0
        x_error_ = x_ref_ - x_base_;
        y_error_ = y_ref_ - y_base_;
        phi_error_ = std::clamp (phi_ref_ - phi_base_, -M_PI, M_PI);
        // RCLCPP_INFO (this->get_logger (), "phi_error = %.5f", phi_error_);

        distance_ = sqrt (x_error_ * x_error_ + y_error_ * y_error_);

        // intermediate:  reference velocities    (v_ref [m/s], w_ref [rad/s])
        v_ref_ = synthesis_7 (distance_, v_base_, a_, stopping_distance_,
                              j_max_, V_MAX_, dt_);
        w_ref_ = synthesis_7 (phi_error_, w_base_, alpha_, stopping_angle_,
                              j_rot_max, W_MAX_, dt_);
        RCLCPP_INFO (this->get_logger (), "v_ref_ = %.2f, w_ref_ = %.2f",
                     v_ref_, w_ref_);

        // output:      reference motor commands  (v_right, v_left) [m/s]
        v_right = v_ref_ + w_ref_;
        v_left = v_ref_ - w_ref_;
        // RCLCPP_INFO (this->get_logger (), "v_right = %.2f, v_left = %.2f",
        // v_right,
        //              v_left);

        // Difference
        dt_ = (time_ns_ - prev_time_) * 0.000000001;
        a_ = (v_base_ - prev_v_) / dt_;
        alpha_ = (w_base_ - prev_w_) / dt_;

        RCLCPP_INFO (this->get_logger (), "a = %.3f", a_);

        // Previous
        prev_v_ = v_base_;
        prev_w_ = w_base_;
        prev_time_ = time_ns_;

        this->publish_motor_cmd ();
      }
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
    // Current
    x_base_ = msg->pose.pose.position.x;
    y_base_ = msg->pose.pose.position.y;
    phi_base_ = tf2::getYaw (msg->pose.pose.orientation);
    v_base_ = msg->twist.twist.linear.x;
    w_base_ = msg->twist.twist.angular.z;
    time_ns_ = rclcpp::Time (msg->header.stamp).nanoseconds ();
    if (!odom_initialized_)
      odom_initialized_ = true;

    // RCLCPP_INFO (this->get_logger (),
    //              "\nv = %.3f\nw = %.3f\nx = %.3f\ny = %.3f\nphi = %.3f",
    //              v_base_, w_base_, x_base_, y_base_, phi_base_);
  }

  double
  synthesis_7 (double distance, double velocity, double acceleration,
               double stopping_distance, double J_MAX, double v_max, double dt)
  {
    double v_ref_ = 0;
    if (dt <= 0 || std::isnan (dt))
      return velocity;

    if (distance <= stopping_distance)
      {
        distance = std::clamp (distance, 0.0, stopping_distance);
        double x = distance / stopping_distance; // Decel phase
        v_ref_ = std::clamp (v_max
                                 * (35.0f * pow (x, 4) - 84.0f * pow (x, 5)
                                    + 70.0f * pow (x, 6) - 20.0f * pow (x, 7)),
                             0.0, velocity);
      }
    else
      {
        if (velocity < v_max * 0.5f)
          v_ref_ = velocity + (acceleration + J_MAX * dt) * dt;
        else
          v_ref_ = velocity + (acceleration - J_MAX * dt) * dt;
      }

    return std::clamp (v_ref_, 0.0, v_max);
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
