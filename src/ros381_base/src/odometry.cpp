#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros381_interfaces/msg/float2.hpp"

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode () : Node ("odometry")
  {
    passive_vel_sub_
        = this->create_subscription<ros381_interfaces::msg::Float2> (
            "passive_vel", 10,
            std::bind (&OdometryNode::callbackPassiveVel, this,
                       std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry> ("odom", 10);

    RCLCPP_INFO (this->get_logger (), "Odometry node is running.");
  }

private:
  void
  callbackPassiveVel (const ros381_interfaces::msg::Float2::SharedPtr msg)
  {
    v = *msg;
    // RCLCPP_INFO (this->get_logger (), "Got passive vel! (%.4f, %.4f)",
    //              v.float2[0], v.float2[1]);
    odometry ();
  }

  void
  odometry ()
  {

    if (odom_initialized_)
      {
        // input:   v (passive wheel velocities)

        // output:  odometry (of base)
        this->publish_odom ();
      }
    else
      {
        odom_initialized_ = true;
        RCLCPP_INFO (this->get_logger(), "Odometry initialized!");
      }
  }

  void
  publish_odom ()
  {
    auto msg = nav_msgs::msg::Odometry ();
    odom_pub_->publish (msg);
  }

  ros381_interfaces::msg::Float2 v;
  bool odom_initialized_ = false;

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
