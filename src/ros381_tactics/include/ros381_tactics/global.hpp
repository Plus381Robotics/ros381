#ifndef ROS381_TACTICS_GLOBAL_HPP
#define ROS381_TACTICS_GLOBAL_HPP

#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include "ros381_tactics/defines.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pybind11/embed.h>

namespace py = pybind11;
using namespace std::placeholders;

class TacticGlobalNode : public rclcpp::Node
{
  public:
    int8_t move_result_ = 0, update_pose_result_ = 0;
    py::module *tactics_module_ = nullptr;

    TacticGlobalNode();

    void send_goal(int type, double x, double y, double phi, int8_t direction, double v_max, double w_max,
                   double distance_tolerance_percentage, double angle_tolerance_percentage, double start_coeff_v,
                   double start_coeff_w, double stop_coeff_v, double stop_coeff_w);
    void update_pose(double x, double y, double phi, uint16_t type);

  private:
    std::unique_ptr<py::scoped_interpreter> guard_;
    unsigned long tick_period_;
    double time_;
    rclcpp::Time start_time_;
    bool match_started_;
    bool chinch_trigger_, chinch_waiting_;
    int8_t global_state_;
	int8_t tactic_side_ = 1;
	uint8_t tactic_num_ = 0;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedPtr pose_client_;
    rclcpp_action::Client<ros381_interfaces::action::Move>::SharedPtr move_client_;
	rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr chinch_waiting_pub_;
	rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr chinch_trigger_sub_;

    py::object tactic_result_;

    void global_fsm();
    void goal_response_callback(
        const rclcpp_action::ClientGoalHandle<ros381_interfaces::action::Move>::SharedPtr &goal_handle);
    void feedback_callback(rclcpp_action::ClientGoalHandle<ros381_interfaces::action::Move>::SharedPtr,
                           const std::shared_ptr<const ros381_interfaces::action::Move::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<ros381_interfaces::action::Move>::WrappedResult &result);
    void pub_time();
	void callback_chinch_state(const example_interfaces::msg::Bool::SharedPtr msg);
	void pub_chinch_waiting();
    void tactic_tick();
    void update_pose_callback(rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedFuture future);
};

void init_python(TacticGlobalNode *node);

#endif // ROS381_TACTICS_GLOBAL_HPP