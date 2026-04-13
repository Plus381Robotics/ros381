#include "../include/signal.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/msg/mini_mbp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <example_interfaces/msg/int8.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <functional>
#include <memory>
#include <thread>

using Move = ros381_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
using namespace std::placeholders;

class MiniMBP : public rclcpp::Node
{
  public:
    MiniMBP() : Node("minimbp")
    {
        this->declare_parameters();

        move_action_server_ = rclcpp_action::create_server<Move>(
            this, "move", std::bind(&MiniMBP::handle_goal, this, _1, _2), std::bind(&MiniMBP::handle_cancel, this, _1),
            std::bind(&MiniMBP::handle_accepted, this, _1));

        obstacle_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
            "obstacle_status", 10, std::bind(&MiniMBP::callback_obstacle, this, _1));

        // obstacle_dir_pub_ = this->create_publisher<example_interfaces::msg::Int8>("obstacle_direction", 10);

        mini_mbp_pub_ = this->create_publisher<ros381_interfaces::msg::MiniMBP>("mini_mbp", 10);

        timer_ =
            this->create_wall_timer(std::chrono::microseconds(period_), std::bind(&MiniMBP::publish_mini_mbp, this));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MiniMBP::callback_odometry, this, _1));

        move_status_budz_sub_ = this->create_subscription<example_interfaces::msg::Int8>(
            "move_status_budz", 10, std::bind(&MiniMBP::move_status_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "MiniMBP node is running with move "
                                        "action server and obstacle handling.");
    }

  private:
    rclcpp_action::Server<Move>::SharedPtr move_action_server_;
    std::shared_ptr<GoalHandleMove> current_goal_handle_;
    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr obstacle_dir_pub_;
    rclcpp::Publisher<ros381_interfaces::msg::MiniMBP>::SharedPtr mini_mbp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr move_status_budz_sub_;

    double x_base_offs_ = 0.0, y_base_offs_ = 0.0, phi_base_offs_ = 0.0;

    bool move_finished = false;
    unsigned short obstacle_ = 0;
    bool obstacle_status_changed_ = false;
    int8_t obstacle_dir_ = 0;
    int movement_state_ = 0;
    uint16_t checksum_;

    // Movement variables (minimal set needed for action server)
    double x_base_ = 0.0, y_base_ = 0.0, phi_base_ = 0.0;
    double x_ref_ = 0.0, y_ref_ = 0.0, phi_ref_ = 0.0;
    double phi_error_ = 0.0, distance_proj_ = 0.0;
    int reg_type_ = 0;
    int direction_ = 1;

    // Tolerance parameters
    double D_TOL_ = 0.005, D_PROJ_TOL_ = 0.002, PHI_TOL_ = 0.002;
    double d_tol_perc_ = 1.0, phi_tol_perc_ = 1.0;

    // Velocity limits
    double V_MAX_ = 1.0, V_MIN_ = 0.1;
    double W_MAX_ = 6.28, W_MIN_ = 0.126;
    double v_max_temp_ = V_MAX_, w_max_temp_ = W_MAX_;

    // Jerk limits
    double J_MAX_ = 40.0, J_MAX_STOP_ = 120.0;
    double J_ROT_MAX_ = 650.0, J_ROT_MAX_STOP_ = 1950.0;
    double j_max_temp_ = J_MAX_, j_rot_max_temp_ = J_ROT_MAX_;

    // Coefficients
    double starting_coeff_v_ = 1.0, stopping_coeff_v_ = 1.0;
    double starting_coeff_w_ = 1.0, stopping_coeff_w_ = 1.0;

    // Control variables
    double v_base_ = 0.0, w_base_ = 0.0;
    double a_ = 0.0, alpha_ = 0.0;
    double dt_ = 0.0;
    double P_w_ = 1.0;
    unsigned long stacked_cnt_ = 0;
    double V_SLOWED_MAX_ = 0.75;

    // Parameters
    double freq_;
    unsigned long period_;

    int8_t move_status_budz_ = 0;

    void move_status_callback(const example_interfaces::msg::Int8::SharedPtr msg)
    {
        move_status_budz_ = msg->data;
    }

    void callback_obstacle(const example_interfaces::msg::UInt8::SharedPtr msg)
    {
        uint8_t new_obstacle_ = msg->data;
        if (new_obstacle_ != obstacle_)
            obstacle_status_changed_ = true;
        else
            obstacle_status_changed_ = false;
        obstacle_ = new_obstacle_;
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Move::Goal> goal)
    {
        (void)uuid;
        if (current_goal_handle_ && current_goal_handle_->is_active() && (goal->type != 0 && goal->type != 10))
        {
            RCLCPP_INFO(get_logger(), "A goal is already active—rejecting new one.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        v_max_temp_ = std::clamp(goal->v_max, V_MIN_, V_MAX_);
        w_max_temp_ = std::clamp(goal->w_max, W_MIN_, W_MAX_);
        starting_coeff_v_ = goal->start_coeff_v;
        stopping_coeff_v_ = goal->stop_coeff_v;
        starting_coeff_w_ = goal->start_coeff_w;
        stopping_coeff_w_ = goal->stop_coeff_w;
        d_tol_perc_ = goal->distance_tolerance_percentage;
        phi_tol_perc_ = goal->angle_tolerance_percentage;
        direction_ = goal->direction;
        
        rclcpp::Time now = this->get_clock()->now();
        uint32_t t = now.nanoseconds();

        checksum_ = (uint16_t)(t ^ (t >> 16));

        switch (goal->type)
        {
        case 0:
            RCLCPP_INFO(this->get_logger(), "Stop.");
            reg_type_ = 0;
            break;
        case 10:
             RCLCPP_INFO(this->get_logger(), "Disassemble.");
            reg_type_ = 10;
            break;
        case -1:
            RCLCPP_INFO(this->get_logger(), "Rotate to PHI:\nphi = %.4f", goal->phi);
            x_ref_ = x_base_;
            y_ref_ = y_base_;
            phi_ref_ = goal->phi;
            reg_type_ = -1;
            break;
        case -2:
            x_ref_ = goal->x;
            y_ref_ = goal->y;
            phi_ref_ = wrap(atan2(goal->y - y_base_, goal->x - x_base_) + (goal->direction - 1) * M_PI * 0.5, -M_PI, M_PI);
            
            RCLCPP_INFO(this->get_logger(), "Rotate from XY:\nx = %.4f, y = %.4f, phi = %.4f", x_base_, y_base_,
                        phi_base_);
            RCLCPP_INFO(this->get_logger(), "Rotate to XY:\nx = %.4f, y = %.4f, phi = %.4f", goal->x, goal->y,
                        phi_ref_);
            reg_type_ = -1;
            break;
        case 1:
            x_ref_ = goal->x;
            y_ref_ = goal->y;
            phi_ref_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Move to XY:\nx = %.4f, y = %.4f", goal->x, goal->y);
            reg_type_ = 1;
            break;
        case 2:
            x_ref_ = x_base_ + goal->direction * goal->y * cos(phi_base_);
            y_ref_ = y_base_ + goal->direction * goal->y * sin(phi_base_);
            phi_ref_ = phi_base_;
            RCLCPP_INFO(this->get_logger(), "Move on Direction:\ndistance = %.4f, direction = %d, phi = %.4f", goal->y,
                        goal->direction, phi_ref_);
            reg_type_ = 1;
            break;
        case 3:
            x_ref_ = x_base_ + goal->direction * goal->y * cos(phi_base_);
            y_ref_ = y_base_ + goal->direction * goal->y * sin(phi_base_);
            phi_ref_ = phi_base_;
            RCLCPP_INFO(this->get_logger(),
                        "Move on Direction Snapped:\ndistance = %.4f, direction "
                        "= %d, phi = %.4f",
                        goal->y, goal->direction, phi_ref_);
            reg_type_ = 1;
            break;
        case 4:
            x_ref_ = x_base_ + goal->direction * goal->y * cos(goal->phi);
            y_ref_ = y_base_ + goal->direction * goal->y * sin(goal->phi);
            phi_ref_ = goal->phi;
            RCLCPP_INFO(this->get_logger(), "Move on Angle:\ndistance = %.4f, direction = %d, phi = %.4f", goal->y,
                        goal->direction, phi_ref_);
            reg_type_ = 1;
            break;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&MiniMBP::move, this, _1), goal_handle}.detach();
    }

    void move(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing movement");
        rclcpp::Rate loop_rate(10);
        auto feedback = std::make_shared<Move::Feedback>();
        auto result = std::make_shared<Move::Result>();
        move_finished = false;
        move_status_budz_ = 0;
        movement_state_ = 0;
        int8_t prev_status = 0;
        // delay of 20ms here
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        while (movement_state_ > -1)
        {
            movement_state_ = move_status_budz_;
            if (goal_handle->is_canceling())
                movement_state_ = -2;

            feedback->angle_remaining = phi_error_;
            feedback->distance_remaininig = distance_proj_;
            goal_handle->publish_feedback(feedback);
            if (prev_status > 0 && move_status_budz_ == 0)
                movement_state_ = -1;
            prev_status = move_status_budz_;
            loop_rate.sleep();
        }
        move_finished = true;
        result->status = movement_state_;
        reset_movement();
        if (rclcpp::ok())
        {
            switch (result->status)
            {
            case -1:
                RCLCPP_INFO(this->get_logger(), "Move succeeded...");
                break;
            case -2:
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Move canceled...");
                return;
            case -3:
                RCLCPP_INFO(this->get_logger(), "Move stacked...");
                break;
            case -4:
                RCLCPP_INFO(this->get_logger(), "Move interrupted by obstacle...");
                break;
            }
            goal_handle->succeed(result);
        }
        else
            result->status = -100;        
    }

    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_base_ = msg->pose.pose.position.x;
        y_base_ = msg->pose.pose.position.y;
        phi_base_ = wrap(tf2::getYaw(msg->pose.pose.orientation), -M_PI, M_PI);
        v_base_ = msg->twist.twist.linear.x;
        w_base_ = msg->twist.twist.angular.z;

        // obstacle_dir_ = get_sign(v_base_);
        // auto dir_msg = example_interfaces::msg::Int8();
        // dir_msg.data = obstacle_dir_;
        // obstacle_dir_pub_->publish(dir_msg);

        // budz
        x_base_offs_ = msg->twist.twist.linear.y;
        y_base_offs_ = msg->twist.twist.linear.z;
        phi_base_offs_ = msg->twist.twist.angular.x;

        // RCLCPP_INFO(this->get_logger(), "ODOM: x=%.3f y=%.3f phi=%.3f | OFFS_IN: x=%.3f y=%.3f phi=%.3f", x_base_,
        //             y_base_, phi_base_, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
        //             msg->twist.twist.angular.x);
    }

    void reset_movement()
    {
        reg_type_ = 0;
        movement_state_ = 0;
        stacked_cnt_ = 0;
        x_ref_ = x_base_;
        y_ref_ = y_base_;
        phi_ref_ = phi_base_;
        direction_ = 1;
        j_max_temp_ = J_MAX_;
        j_rot_max_temp_ = J_ROT_MAX_;
        v_max_temp_ = V_MAX_;
        w_max_temp_ = W_MAX_;
        starting_coeff_v_ = 1.0;
        stopping_coeff_v_ = 1.0;
        starting_coeff_w_ = 1.0;
        stopping_coeff_w_ = 1.0;
        d_tol_perc_ = 1.0;
        phi_tol_perc_ = 1.0;
    }


    double wrap(double signal, double min, double max)
    {
        double temp = signal;
        wrap_ptr(&temp, min, max);
        return temp;
    }

    void wrap_ptr(double *signal, double min, double max)
    {
        double diff = max - min;
        while (*signal > max)
            *signal -= diff;
        while (*signal < min)
            *signal += diff;
    }

    void publish_mini_mbp()
    {
        // if (move_finished == false)
        // {
        auto msg = ros381_interfaces::msg::MiniMBP();
        msg.type = reg_type_;
        float dx = x_ref_ - x_base_offs_;
        float dy = y_ref_ - y_base_offs_;

        float c = cos(phi_base_offs_);
        float s = sin(phi_base_offs_);

        msg.x = c * dx + s * dy;
        msg.y = -s * dx + c * dy;
        msg.phi = wrap(phi_ref_ - phi_base_offs_, -M_PI, M_PI);
        msg.direction = direction_;
        msg.obstacle = obstacle_;
        msg.v_max_100 = (uint8_t)(v_max_temp_ * 100.0f);
        msg.w_max_10 = (uint8_t)(w_max_temp_ * 10.0f);
        // TODO: ovo uradi kako treba
        msg.tol_perc = 255;
        msg.coeff = 255;

        // uint16_t checksum = fletcher16();
        
        msg.checksum = checksum_;

        // RCLCPP_INFO(this->get_logger(),
        //             "REF: x=%.3f y=%.3f phi=%.3f | "
        //             "BASE: x=%.3f y=%.3f phi=%.3f | "
        //             "DIFF: dx=%.3f dy=%.3f dphi=%.3f | "
        //             "MSG: x=%.3f y=%.3f phi=%.3f",
        //             x_ref_, y_ref_, phi_ref_, x_base_, y_base_, phi_base_, dx, dy, phi_ref_ - phi_base_offs_,
        //             msg.x, msg.y, msg.phi);
        mini_mbp_pub_->publish(msg);
        // }
    }

    void declare_parameters()
    {
        this->declare_parameter("FREQ", 10.0);
        freq_ = this->get_parameter("FREQ").as_double();
        period_ = 1000000 / freq_;

        this->declare_parameter("V_MAX", 1.0);
        V_MAX_ = this->get_parameter("V_MAX").as_double();
        this->declare_parameter("V_MIN", 0.1);
        V_MIN_ = this->get_parameter("V_MIN").as_double();
        this->declare_parameter("W_MAX", 6.28);
        W_MAX_ = this->get_parameter("W_MAX").as_double();
        this->declare_parameter("W_MIN", 0.126);
        W_MIN_ = this->get_parameter("W_MIN").as_double();
        this->declare_parameter("P_w", 1.0);
        P_w_ = this->get_parameter("P_w").as_double();
        this->declare_parameter("J_MAX", 40.0);
        J_MAX_ = this->get_parameter("J_MAX").as_double();
        this->declare_parameter("J_MAX_STOP", 120.0);
        J_MAX_STOP_ = this->get_parameter("J_MAX_STOP").as_double();
        this->declare_parameter("J_ROT_MAX", 650.0);
        J_ROT_MAX_ = this->get_parameter("J_ROT_MAX").as_double();
        this->declare_parameter("J_ROT_MAX_STOP", 1950.0);
        J_ROT_MAX_STOP_ = this->get_parameter("J_ROT_MAX_STOP").as_double();
        this->declare_parameter("D_TOL", 0.005);
        D_TOL_ = this->get_parameter("D_TOL").as_double();
        this->declare_parameter("D_PROJ_TOL", 0.002);
        D_PROJ_TOL_ = this->get_parameter("D_PROJ_TOL").as_double();
        this->declare_parameter("PHI_TOL", 0.002);
        PHI_TOL_ = this->get_parameter("PHI_TOL").as_double();

        v_max_temp_ = V_MAX_;
        w_max_temp_ = W_MAX_;
        j_max_temp_ = J_MAX_;
        j_rot_max_temp_ = J_ROT_MAX_;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MiniMBP>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}