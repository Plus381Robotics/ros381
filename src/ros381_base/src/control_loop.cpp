// #include "geometry_msgs/msg/pose2_d.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "rclcpp/rclcpp.hpp"

class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode () : Node ("control_loop")
  {
    this->declare_parameter ("freq_hz", 10.0);
    freq_hz_ = this->get_parameter ("freq_hz").as_double ();
    period_us_ = 1000000 / freq_hz_;

    this->declare_parameter ("p", 1.0);
    p_ = this->get_parameter ("p").as_double ();
    this->declare_parameter ("i", 0.0);
    i_ = this->get_parameter ("i").as_double ();
    this->declare_parameter ("d", 0.0);
    d_ = this->get_parameter ("d").as_double ();

    // TODO:
    // - uzima mete: action
    timer_ = this->create_wall_timer (
        std::chrono::microseconds (period_us_),
        std::bind (&ControlLoopNode::control_loop, this));
    motor_cmd_publisher_
        = this->create_publisher<ros381_interfaces::msg::Float2> (
            "motor_cmd", 10);

    RCLCPP_INFO (this->get_logger (), "Control loop node is running.");
  }

private:
  double p_, i_, d_;
  double freq_hz_;
  int64_t period_us_;
  double x_ref, y_ref, phi_ref;
  double w_right = 10, w_left = 10;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros381_interfaces::msg::Float2>::SharedPtr
      motor_cmd_publisher_;

  void
  control_loop ()
  {
    // input:   reference velocities            (vx, vy, w)

    // output:  reference motor commands  (ω60, ω180, ω300)
    this->publish_motor_cmd ();
  }

  void
  publish_motor_cmd ()
  {
    auto msg = ros381_interfaces::msg::Float2 ();
    msg.float2[0] = w_right;
    msg.float2[1] = w_left;
    motor_cmd_publisher_->publish (msg);
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
