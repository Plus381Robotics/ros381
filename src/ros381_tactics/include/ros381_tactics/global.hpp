#ifndef ROS381_TACTICS_GLOBAL_HPP
#define ROS381_TACTICS_GLOBAL_HPP

#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include "ros381_tactics/defines.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pybind11/embed.h>

namespace py = pybind11;

class TacticGlobalNode : public rclcpp::Node
{
  public:
    int8_t move_result_ = 0;
    py::module *tactics_module_ = nullptr;

    TacticGlobalNode();

    void send_goal(int type, double x, double y, double phi, int8_t direction, double v_max, double w_max,
                   double distance_tolerance_percentage, double angle_tolerance_percentage, double start_coeff_v,
                   double start_coeff_w, double stop_coeff_v, double stop_coeff_w);

  private:
    std::unique_ptr<py::scoped_interpreter> guard_;
    unsigned long tick_period_;
    double time_;
    rclcpp::Time start_time_;
    bool match_started_;
    bool chich_trigger_, chich_waiting_;
    int8_t global_state_;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr chich_service_;
    rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedPtr pose_client_;
    rclcpp_action::Client<ros381_interfaces::action::Move>::SharedPtr move_client_;

    py::object tactic_result_;

    void global_fsm();
    void goal_response_callback(
        const rclcpp_action::ClientGoalHandle<ros381_interfaces::action::Move>::SharedPtr &goal_handle);
    void feedback_callback(rclcpp_action::ClientGoalHandle<ros381_interfaces::action::Move>::SharedPtr,
                           const std::shared_ptr<const ros381_interfaces::action::Move::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<ros381_interfaces::action::Move>::WrappedResult &result);
    void pub_time();
    void chich_trigger(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
                       std::shared_ptr<example_interfaces::srv::Trigger::Response> response);
    void tactic_tick();
    int update_pose(double x, double y, double phi, uint16_t type);
    void update_pose_callback(rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedFuture future);
};

void init_python(TacticGlobalNode *node);

#endif // ROS381_TACTICS_GLOBAL_HPP