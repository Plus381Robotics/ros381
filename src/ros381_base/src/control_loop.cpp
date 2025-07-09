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
    period_ = 1000000 / freq_;

    this->declare_parameter ("L", 0.1545);
    L_ = this->get_parameter ("L").as_double ();

    // TODO:
    // - uzima mete: action
    timer_ = this->create_wall_timer (
        std::chrono::microseconds (period_),
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
  double L_;                                                       // [m]
  double phi_base_, phi_error_, PHI_TOL_ = 0.009, phi_ref_ = 3.14; // [rad]
  double x_base_, x_error_, x_ref_ = -1.1;                         // [m]
  double y_base_, y_error_, y_ref_ = -0.5;                         // [m]
  // TODO: povecaj max brzine
  double v_base_, V_MAX_ = 2.0, V_MIN_ = 0.04, v_ref_ = 0.0, prev_v_; // [m/s]
  double w_base_, W_MAX_ = 12.6, W_MIN_ = 0.251, w_ref_ = 0.0,
                  prev_w_;     // [rad/s]
  double v_max_temp_ = V_MAX_; // [m/s]
  double w_max_temp_ = W_MAX_; // [rad/s]
  double distance_, distance_proj_, D_TOL_ = 0.005, D_LONG_TOL_ = 0.08,
                                    D_PROJ_TOL_ = 0.002; // [m]
  double stopping_distance_ = 0, starting_distance_ = 0; // [m]
  double stopping_angle_ = 0, starting_angle_ = 0;       // [rad]
  double stopping_coeff_w_ = 1.0, starting_coeff_w_ = 1.0;
  double stopping_coeff_v_ = 1.0, starting_coeff_v_ = 1.0;
  double slowing_coeff_ = 1.0;
  double a_, alpha_; // [m/s^2], [rad/s^2]
  double J_MAX_ = 40.0, j_max_temp_ = J_MAX_, j_max_stop_ = 120.0; // [m/s^3]
  double J_ROT_MAX_ = 650.0, j_rot_max_temp_ = J_ROT_MAX_,
         j_rot_max_stop_ = 1950.0;    // [rad/s^3]
  unsigned long time_ns_, prev_time_; // [ns]
  double dt_;                         // [s]
  double freq_;                       // [Hz]
  short reg_type_ = 1, reg_phase_ = 0, movement_state_ = 0, direction_ = -1;
  unsigned long period_;              // [us]
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
        switch (reg_type_)
          {
          case -1:
            rotate ();
            break;
          case 0:
            v_ref_ = 0;
            w_ref_ = 0;
            break;
          case 1:
            go_to_xy ();
            break;
          }

        // output:      reference motor commands  (v_right, v_left) [m/s]
        v_right = v_ref_ + w_ref_ * L_ * 0.5;
        v_left = v_ref_ - w_ref_ * L_ * 0.5;
        // RCLCPP_INFO (this->get_logger (), "v_right = %.2f, v_left = %.2f",
        // v_right,
        //              v_left);

        // Difference
        dt_ = (time_ns_ - prev_time_) * 0.000000001;
        a_ = (v_base_ - prev_v_) / dt_;
        alpha_ = (w_base_ - prev_w_) / dt_;

        // Previous
        prev_v_ = v_base_;
        prev_w_ = w_base_;
        prev_time_ = time_ns_;

        this->publish_motor_cmd ();
      }
  }

  void
  rotate ()
  {
    if (movement_state_ == 0)
      {
        starting_angle_
            = 5 * pow (w_max_temp_, 1.5) / 3 / sqrt (j_rot_max_temp_);
        starting_angle_ *= starting_coeff_w_;
        stopping_angle_
            = 5 * pow (w_max_temp_, 1.5) / 3 / sqrt (j_rot_max_stop_);
        stopping_angle_ *= stopping_coeff_w_;

        slowing_coeff_ = std::clamp (
            pow (phi_error_ / (starting_angle_ + stopping_angle_), 2.0 / 3.0),
            0.0, 1.0);

        w_max_temp_ *= slowing_coeff_;
        stopping_angle_ = 5 * pow (w_max_temp_, 1.5) / 3
                          / sqrt (j_rot_max_stop_) * stopping_coeff_w_;

        movement_state_ = 1;
      }

    phi_error_ = wrap (phi_ref_ - phi_base_, -M_PI, M_PI);
    v_ref_ = 0;
    w_ref_ = synthesis_7 (phi_error_, w_base_, alpha_, j_rot_max_temp_,
                          stopping_angle_, w_max_temp_, W_MIN_, dt_);
    // RCLCPP_INFO (this->get_logger (), "Phi error = %.4f, Phitolerance =
    // %.3f",
    //              phi_error_, PHI_TOL_);
    if (fabs (phi_error_) < PHI_TOL_)
      {
        reg_type_ = 0;
        w_max_temp_ = W_MAX_;
        j_rot_max_temp_ = J_ROT_MAX_;
        movement_state_ = -1;
      }
  }

  void
  go_to_xy ()
  {
    if (movement_state_ == 0)
      {
        movement_state_ = 1;
      }

    x_error_ = x_ref_ - x_base_;
    y_error_ = y_ref_ - y_base_;
    phi_error_ = wrap (atan2 (y_error_, x_error_) - phi_base_
                           + (direction_ - 1) * M_PI * 0.5,
                       -M_PI, M_PI);
    switch (reg_phase_)
      {
      case 0:
        starting_angle_
            = 5 * pow (w_max_temp_, 1.5) / 3 / sqrt (j_rot_max_temp_);
        starting_angle_ *= starting_coeff_w_;
        stopping_angle_
            = 5 * pow (w_max_temp_, 1.5) / 3 / sqrt (j_rot_max_stop_);
        stopping_angle_ *= stopping_coeff_w_;

        slowing_coeff_ = std::clamp (
            pow (phi_error_ / (starting_angle_ + stopping_angle_), 2.0 / 3.0),
            0.0, 1.0);

        // Calculate new parameters
        w_max_temp_ *= slowing_coeff_;
        stopping_angle_ = 5 * pow (w_max_temp_, 1.5) / 3
                          / sqrt (j_rot_max_stop_) * stopping_coeff_w_;

        reg_phase_ = 1;
        break;
      case 1:
        v_ref_ = 0;
        w_ref_ = synthesis_7 (phi_error_, w_base_, alpha_, j_rot_max_temp_,
                              stopping_angle_, w_max_temp_, W_MIN_, dt_);
        if (fabs (phi_error_) < PHI_TOL_)
          {
            reg_phase_ = 2;
            w_max_temp_ = W_MAX_;
          }
        break;
      case 2:
        distance_ = sqrt (x_error_ * x_error_ + y_error_ * y_error_);
        distance_proj_ = distance_ * cos (phi_error_);

        starting_distance_ = 5 * pow (v_max_temp_, 1.5) / 3
                             / sqrt (j_max_temp_) * starting_coeff_v_;
        stopping_distance_ = 5 * pow (v_max_temp_, 1.5) / 3
                             / sqrt (j_max_stop_) * stopping_coeff_v_;

        slowing_coeff_ = std::clamp (
            pow (distance_proj_ / (starting_distance_ + stopping_distance_),
                 2.0 / 3.0),
            0.0, 1.0);

        // Calculate new parameters
        v_max_temp_ *= slowing_coeff_;
        stopping_distance_ = 5 * pow (v_max_temp_, 1.5) / 3
                             / sqrt (j_max_stop_) * stopping_coeff_v_;

        reg_phase_ = 3;
        break;
      case 3:
        distance_ = sqrt (x_error_ * x_error_ + y_error_ * y_error_);
        distance_proj_ = distance_ * cos (phi_error_);

        v_ref_ = synthesis_7 (distance_proj_ * direction_, v_base_, a_,
                              j_max_temp_, stopping_distance_, v_max_temp_,
                              V_MIN_, dt_);
        if (distance_proj_ > D_LONG_TOL_)
          {
            // TODO: P regulator ovde umesto ove sinteze
            w_ref_ = synthesis_7 (phi_error_, w_base_, alpha_, j_rot_max_temp_,
                                  stopping_angle_, w_max_temp_, 0.0, dt_);
          }
        else
          w_ref_ = 0;

        // RCLCPP_INFO (this->get_logger (),
        //              "Distance = %.4f, Distance tolerance = %.3f",
        //              distance_, D_TOL_);
        // RCLCPP_INFO (
        //     this->get_logger (),
        //     "Distance projected = %.4f, Distance projected tolerance =
        //     %.3f", distance_proj_, D_PROJ_TOL_);
        if (distance_proj_ < D_PROJ_TOL_ && fabs (distance_) < D_TOL_)
          {
            reg_type_ = 0;
            w_max_temp_ = W_MAX_;
            v_max_temp_ = V_MAX_;
            movement_state_ = -1;
          }

        break;
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
               double J_MAX, double stopping_distance, double v_max,
               double v_min, double dt)
  {
    double abs_distance = fabs (distance);
    double abs_velocity = fabs (velocity);
    double abs_acceleration = fabs (acceleration);
    double v_ref = 0;
    if (dt <= 0 || std::isnan (dt))
      return 0.0;

    if (abs_distance <= stopping_distance)
      {
        double x = abs_distance / stopping_distance; // Decel phase
        v_ref = v_max
                * (35.0f * pow (x, 4) - 84.0f * pow (x, 5) + 70.0f * pow (x, 6)
                   - 20.0f * pow (x, 7));
        // v_ref = std::clamp (v_ref, 0.0, abs_velocity);
        // v_ref = 0;
      }
    else
      {
        double j_step = J_MAX * dt;
        if (abs_velocity < v_max * 0.5f)
          v_ref = abs_velocity + (abs_acceleration + j_step) * dt;
        else if (j_step < abs_acceleration * 1.05)
          v_ref = abs_velocity + (abs_acceleration - j_step) * dt;
        else
          v_ref = v_max;
      }
    v_ref = std::clamp (v_ref, v_min, v_max);

    return std::clamp (get_sign (distance) * v_ref, -v_max, v_max);
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
