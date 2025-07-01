#include "nav_msgs/msgs/odometry.hpp"
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
  rclcpp::Subscription<ros381_interfaces::msg::Float2>::SharedPtr
      passive_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  void
  callbackPassiveVel (const ros381_interfaces::msg::Float2::SharedPtr msg)
  {
    RCLCPP_INFO (this->get_logger (), "Got passive vel! (%.2f, %.2f)", msg.float2[0], msg.float2[1]);
    // TODO: ovde da pozove odometriju
  }

  void
  odometry ()
  {
    // input:   float2 (passive vel)

    // output:  odometry (of base)
    this->publish_odom ();
  }

  void
  publish_odom ()
  {
    auto msg = nav_msgs::msg::Odometry ();
    odom_pub_->publish (msg);
  }
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
