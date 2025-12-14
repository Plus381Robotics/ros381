#ifndef ROS381_TACTICS_GLOBAL_HPP
#define ROS381_TACTICS_GLOBAL_HPP

#include "dynamixel_sdk_custom_interfaces/action/ax_bulk_move.hpp"
#include "dynamixel_sdk_custom_interfaces/action/ax_hybrid_move.hpp"
#include "dynamixel_sdk_custom_interfaces/action/ax_move.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/msg/float3.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include "ros381_tactics/defines.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pybind11/embed.h>

namespace py = pybind11;
using namespace std::placeholders;
using Move = ros381_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;
using AxMove = dynamixel_sdk_custom_interfaces::action::AxMove;
using AxMoveGoalHandle = rclcpp_action::ClientGoalHandle<AxMove>;
using AxBulkMove = dynamixel_sdk_custom_interfaces::action::AxBulkMove;
using AxBulkMoveGoalHandle = rclcpp_action::ClientGoalHandle<AxBulkMove>;
using AxHybridMove = dynamixel_sdk_custom_interfaces::action::AxHybridMove;
using AxHybridMoveGoalHandle = rclcpp_action::ClientGoalHandle<AxHybridMove>;

struct AxMoveGoal
{
    uint8_t id;
    uint16_t position;
    uint16_t velocity;
    uint16_t position_tolerance;
};

class TacticGlobalNode : public rclcpp::Node
{
  public:
    int8_t move_result_ = 0, update_pose_result_ = 0;
    int8_t ax_move_result_ = 0;
    py::module *tactics_module_ = nullptr;

    TacticGlobalNode();

    void send_goal(int type, double x, double y, double phi, int8_t direction, double v_max, double w_max,
                   double distance_tolerance_percentage, double angle_tolerance_percentage, double start_coeff_v,
                   double start_coeff_w, double stop_coeff_v, double stop_coeff_w);
    void cancel_goal();
    void update_pose(double x, double y, double phi, uint16_t type);
    void publish_pose_offset(double x, double y, double phi);
    void ax_move_goal(AxMoveGoal goal);
    void ax_bulk_move_goal(std::vector<AxMoveGoal> goals);
    void ax_hybrid_move_goal(uint8_t id, uint16_t velocity, float zero_time, int8_t direction, uint16_t delta_pos);

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
    bool reset_on_, reset_was_on_ = false;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedPtr pose_client_;
    rclcpp_action::Client<ros381_interfaces::action::Move>::SharedPtr move_client_;
    rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr chinch_waiting_pub_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr chinch_trigger_sub_;
    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr switches_sub_;
    rclcpp::Publisher<ros381_interfaces::msg::Float3>::SharedPtr pose_offs_pub_;
    rclcpp_action::Client<dynamixel_sdk_custom_interfaces::action::AxMove>::SharedPtr ax_move_client_;
    rclcpp_action::Client<dynamixel_sdk_custom_interfaces::action::AxBulkMove>::SharedPtr ax_bulk_move_client_;
    rclcpp_action::Client<dynamixel_sdk_custom_interfaces::action::AxHybridMove>::SharedPtr ax_hybrid_move_client_;

    py::object tactic_result_;

    void global_fsm();
    void goal_response_callback(
        const GoalHandleMove::SharedPtr &goal_handle);
    void feedback_callback(GoalHandleMove::SharedPtr,
                           const std::shared_ptr<const Move::Feedback> feedback);
    void result_callback(const GoalHandleMove::WrappedResult &result);
    void pub_time();
    void callback_chinch_state(const example_interfaces::msg::Bool::SharedPtr msg);
    void pub_chinch_waiting();
    void tactic_tick();
    void update_pose_callback(rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedFuture future);
    void callback_switches(const example_interfaces::msg::UInt8::SharedPtr msg);

    void ax_move_goal_response_callback(const AxMoveGoalHandle::SharedPtr &goal_handle);
    void ax_move_feedback_callback(AxMoveGoalHandle::SharedPtr, const std::shared_ptr<const AxMove::Feedback> feedback);
    void ax_move_result_callback(const AxMoveGoalHandle::WrappedResult &result);
};

void init_python(TacticGlobalNode *node);

#endif // ROS381_TACTICS_GLOBAL_HPP