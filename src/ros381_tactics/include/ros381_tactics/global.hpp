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
#include "ros381_interfaces/msg/crate.hpp"
#include "ros381_interfaces/msg/crate_stack.hpp"
#include "ros381_interfaces/msg/float3.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include "ros381_tactics/defines.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
    int8_t ax_move_result_ = 0, ax_hybrid_move_result_ = 0, ax_bulk_move_result_ = 0;
    uint16_t ax_hybrid_end_position_ = 0;
    py::module *tactics_module_ = nullptr;

    // AX ids:
    uint8_t lift_front_id_ = 15, lift_back_id_ = 5;
    uint8_t clan1_front_id_ = 11, clan2_front_id_ = 12, clan3_front_id_ = 13, clan4_front_id_ = 14;
    uint8_t clan1_back_id_ = 1, clan2_back_id_ = 2, clan3_back_id_ = 3, clan4_back_id_ = 4;
    uint8_t cursor_id_ = 6;
    // AX positions:
    uint16_t lift_up_pos_ = 900, lift_down_pos_ = 300, lift_carry_pos_ = 400, lift_rotating_pos_ = 750, lift_dropoff_pos_ = 450;
    uint16_t cursor_up_pos_ = 950;
    uint16_t clanL_up_pos_ = 701, clanL_down_pos_ = 0, clanR_up_pos_ = 322, clanR_down_pos_ = 1023;
    uint16_t clanL_undep_pos_ = 511, clanR_undep_pos_ = 511;

    // CrateStacks:
    double cs_front_x = 9.9, cs_front_y = 9.9, cs_front_phi = 9.9;
    double cs_back_x = 9.9, cs_back_y = 9.9, cs_back_phi = 9.9;
    bool cs_front_full = false, cs_back_full = false;
    // int8_t crates_back_[4] = {-1, -1, -1, -1};
    std::array<int8_t, 4> crates_back_ = {-1, -1, -1, -1};
    std::array<int8_t, 4> crates_front_ = {-1, -1, -1, -1};
    // int8_t crates_front_[4] = {-1, -1, -1, -1};
    bool consuming_front_ = false, consuming_back_ = false;
    double x_base_ = 0.0, y_base_ = 0.0, phi_base_ = 0.0, v_base_ = 0.0, w_base_ = 0.0;

    TacticGlobalNode();

    void send_goal(int type, double x, double y, double phi, int8_t direction, double v_max, double w_max,
                   double distance_tolerance_percentage, double angle_tolerance_percentage, double start_coeff_v,
                   double start_coeff_w, double stop_coeff_v, double stop_coeff_w);
    void cancel_goal();
    void update_pose(double x, double y, double phi, uint16_t type);
    void publish_pose_offset(double x, double y, double phi);
    void ax_move_goal(AxMoveGoal goal);
    void ax_bulk_move_goal(const std::vector<AxMoveGoal> &goals);
    void ax_hybrid_move_goal(uint8_t id, uint16_t velocity, float zero_time, int16_t delta_pos);
    void set_vacuum(bool front, bool back);
    void add_vacuum(bool front, bool back);
    void remove_vacuum(bool front, bool back);
    uint8_t vacuum_mask(bool front, bool back);

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
    uint8_t vacuum_ = 0;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedPtr pose_client_;
    rclcpp_action::Client<ros381_interfaces::action::Move>::SharedPtr move_client_;
    rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr chinch_waiting_pub_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr chinch_trigger_sub_;
    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr switches_sub_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr vacuum_pub_;
    rclcpp::Publisher<ros381_interfaces::msg::Float3>::SharedPtr pose_offs_pub_;
    rclcpp_action::Client<dynamixel_sdk_custom_interfaces::action::AxMove>::SharedPtr ax_move_client_;
    rclcpp_action::Client<dynamixel_sdk_custom_interfaces::action::AxBulkMove>::SharedPtr ax_bulk_move_client_;
    rclcpp_action::Client<dynamixel_sdk_custom_interfaces::action::AxHybridMove>::SharedPtr ax_hybrid_move_client_;
    rclcpp::Subscription<ros381_interfaces::msg::CrateStack>::SharedPtr crate_stack_front_sub_;
    rclcpp::Subscription<ros381_interfaces::msg::CrateStack>::SharedPtr crate_stack_back_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    py::object tactic_result_;

    void global_fsm();
    void goal_response_callback(const GoalHandleMove::SharedPtr &goal_handle);
    void feedback_callback(GoalHandleMove::SharedPtr, const std::shared_ptr<const Move::Feedback> feedback);
    void result_callback(const GoalHandleMove::WrappedResult &result);
    void pub_time();
    void callback_chinch_state(const example_interfaces::msg::Bool::SharedPtr msg);
    void pub_chinch_waiting();
    void tactic_tick();
    void update_pose_callback(rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedFuture future);
    void callback_switches(const example_interfaces::msg::UInt8::SharedPtr msg);
    void callback_crate_stack_back(const ros381_interfaces::msg::CrateStack msg);
    void callback_crate_stack_front(const ros381_interfaces::msg::CrateStack msg);

    void ax_move_goal_response_callback(const AxMoveGoalHandle::SharedPtr &goal_handle);
    void ax_move_feedback_callback(AxMoveGoalHandle::SharedPtr, const std::shared_ptr<const AxMove::Feedback> feedback);
    void ax_move_result_callback(const AxMoveGoalHandle::WrappedResult &result);
    void ax_bulk_move_goal_response_callback(const AxBulkMoveGoalHandle::SharedPtr &goal_handle);
    void ax_bulk_move_feedback_callback(AxBulkMoveGoalHandle::SharedPtr,
                                        const std::shared_ptr<const AxBulkMove::Feedback> feedback);
    void ax_bulk_move_result_callback(const AxBulkMoveGoalHandle::WrappedResult &result);
    void ax_hybrid_move_goal_response_callback(const AxHybridMoveGoalHandle::SharedPtr &goal_handle);
    void ax_hybrid_move_feedback_callback(AxHybridMoveGoalHandle::SharedPtr,
                                          const std::shared_ptr<const AxHybridMove::Feedback> feedback);
    void ax_hybrid_move_result_callback(const AxHybridMoveGoalHandle::WrappedResult &result);
    void declare_ax_params();
    uint8_t crate_position(double x);
    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};

void init_python(TacticGlobalNode *node);

#endif // ROS381_TACTICS_GLOBAL_HPP