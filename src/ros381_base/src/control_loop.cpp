#include "../include/signal.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "tf2/utils.h"
#include <functional>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

using Move = ros381_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
using namespace std::placeholders;

class ControlLoopNode : public rclcpp::Node
{
  public:
    ControlLoopNode() : Node("control_loop")
    {

        this->declare_parameters();
        timer_ = this->create_wall_timer(std::chrono::microseconds(period_),
                                         std::bind(&ControlLoopNode::control_loop, this));
        motor_cmd_publisher_ = this->create_publisher<ros381_interfaces::msg::Float2>("motor_cmd", 10);
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ControlLoopNode::callback_odometry, this, _1));
        move_action_server_ =
            rclcpp_action::create_server<Move>(this, "move", std::bind(&ControlLoopNode::handle_goal, this, _1, _2),
                                               std::bind(&ControlLoopNode::handle_cancel, this, _1),
                                               std::bind(&ControlLoopNode::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Control loop node is running.");
    }

  private:
    double L_;                                              // [m]
    double phi_base_, phi_error_, PHI_TOL_, phi_ref_ = 0.0; // [rad]
    double x_base_, x_error_, x_ref_ = 0.0;                 // [m]
    double y_base_, y_error_, y_ref_ = 0.0;                 // [m]
    double v_base_, V_MAX_, V_MIN_, v_ref_ = 0.0, prev_v_;  // [m/s]
    double MOTOR_V_MAX_;                                    // [m/s]
    double w_base_, W_MAX_, W_MIN_, w_ref_ = 0.0,
                                    prev_w_; // [rad/s]
    double v_max_temp_;                      // [m/s]
    double w_max_temp_;                      // [rad/s]
    double distance_, distance_proj_;        // [m]
    double D_TOL_, D_LONG_TOL_, D_PROJ_TOL_,
        D_SHORT_TOL_; // [m]
    double d_tol_perc_ = 1.0, phi_tol_perc_ = 1.0;
    double stopping_distance_ = 0, starting_distance_ = 0; // [m]
    double stopping_angle_ = 0, starting_angle_ = 0;       // [rad]
    double stopping_coeff_w_ = 1.0, starting_coeff_w_ = 1.0;
    double stopping_coeff_v_ = 1.0, starting_coeff_v_ = 1.0;
    double slowing_coeff_ = 1.0;
    double P_w_;
    double a_, alpha_;                       // [m/s^2], [rad/s^2]
    double J_MAX_, j_max_temp_, J_MAX_STOP_; // [m/s^3]
    double J_ROT_MAX_, j_rot_max_temp_,
        J_ROT_MAX_STOP_;                // [rad/s^3]
    unsigned long time_ns_, prev_time_; // [ns]
    double dt_;                         // [s]
    double freq_;                       // [Hz]
    short reg_type_ = 0, reg_phase_ = 0, movement_state_ = 0, direction_ = 1;
    unsigned long period_;                // [us]
    double v_right_ = 0.0, v_left_ = 0.0; // [m/s]
    bool odom_initialized_ = false;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros381_interfaces::msg::Float2>::SharedPtr motor_cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp_action::Server<Move>::SharedPtr move_action_server_;
    std::shared_ptr<GoalHandleMove> current_goal_handle_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Move::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received move request of type: %d", goal->type);
        (void)uuid;
        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            RCLCPP_INFO(get_logger(), "A goal is already active—rejecting new one.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // TODO:
        // postavi reference ovde na osnovu tipa kretnje
        // limituj ogranicenja
        // rclcpp_info
        switch (goal->type)
        {
        // Move to XY
        case 1:
            x_ref_ = goal->x;
            y_ref_ = goal->y;
            phi_ref_ = 0.0;
            direction_ = goal->direction;
            v_max_temp_ = goal->v_max;
            w_max_temp_ = goal->w_max;
            starting_coeff_v_ = goal->start_coeff_v;
            stopping_coeff_v_ = goal->stop_coeff_v;
            starting_coeff_w_ = goal->start_coeff_w;
            stopping_coeff_w_ = goal->stop_coeff_w;
            d_tol_perc_ = goal->distance_tolerance_percentage;
            phi_tol_perc_ = goal->angle_tolerance_percentage;
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
        std::thread{std::bind(&ControlLoopNode::move, this, _1), goal_handle}.detach();
    }

    void move(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing movement");
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Move::Feedback>();
        auto result = std::make_shared<Move::Result>();

        while (movement_state_ > -1)
        {
            if (goal_handle->is_canceling())
            {
                result->status = -2;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            feedback->angle_remaining = phi_error_;
            feedback->distance_remaininig = distance_proj_;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            movement_state_ = 0;
            result->status = -1;
            x_ref_ = x_base_;
            y_ref_ = y_base_;
            phi_ref_ = phi_base_;
            direction_ = 1;
            v_max_temp_ = V_MAX_;
            w_max_temp_ = W_MAX_;
            starting_coeff_v_ = 1.0;
            stopping_coeff_v_ = 1.0;
            starting_coeff_w_ = 1.0;
            stopping_coeff_w_ = 1.0;
            d_tol_perc_ = 1.0;
            phi_tol_perc_ = 1.0;
            reg_type_ = 0;
            reg_phase_ = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void control_loop()
    {
        if (odom_initialized_)
        {
            switch (reg_type_)
            {
            case -1:
                rotate();
                break;
            case 0:
                v_ref_ = 0;
                w_ref_ = 0;
                break;
            case 1:
                go_to_xy();
                break;
            }

            v_right_ = std::clamp(v_ref_ + w_ref_ * L_ * 0.5, -MOTOR_V_MAX_, MOTOR_V_MAX_);
            v_left_ = std::clamp(v_ref_ - w_ref_ * L_ * 0.5, -MOTOR_V_MAX_, MOTOR_V_MAX_);
            scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);

            dt_ = (time_ns_ - prev_time_) * 0.000000001;
            a_ = (v_base_ - prev_v_) / dt_;
            alpha_ = (w_base_ - prev_w_) / dt_;

            prev_v_ = v_base_;
            prev_w_ = w_base_;
            prev_time_ = time_ns_;

            this->publish_motor_cmd();
        }
    }

    void rotate()
    {
        if (movement_state_ == 0)
        {
            starting_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(j_rot_max_temp_) * starting_coeff_w_;
            stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_) * stopping_coeff_w_;

            slowing_coeff_ =
                std::clamp(pow(fabs(phi_error_) / (starting_angle_ + stopping_angle_), 2.0 / 3.0), 0.0, 1.0);

            w_max_temp_ *= slowing_coeff_;
            stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_) * stopping_coeff_w_;

            movement_state_ = 1;
        }

        phi_error_ = wrap(phi_ref_ - phi_base_, -M_PI, M_PI);
        v_ref_ = 0;
        w_ref_ = synthesis_7(phi_error_, w_base_, alpha_, j_rot_max_temp_, stopping_angle_, w_max_temp_, W_MIN_, dt_);

        if (fabs(phi_error_) < PHI_TOL_ * phi_tol_perc_)
        {
            reg_type_ = 0;
            w_max_temp_ = W_MAX_;
            j_rot_max_temp_ = J_ROT_MAX_;
            movement_state_ = -1;
        }
    }

    void go_to_xy()
    {
        if (movement_state_ == 0)
        {
            movement_state_ = 1;
        }

        x_error_ = x_ref_ - x_base_;
        y_error_ = y_ref_ - y_base_;
        phi_error_ = wrap(atan2(y_error_, x_error_) - phi_base_ + (direction_ - 1) * M_PI * 0.5, -M_PI, M_PI);
        switch (reg_phase_)
        {
        case 0:
            starting_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(j_rot_max_temp_) * starting_coeff_w_;
            stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_) * stopping_coeff_w_;

            slowing_coeff_ =
                std::clamp(pow(fabs(phi_error_) / (starting_angle_ + stopping_angle_), 2.0 / 3.0), 0.0, 1.0);

            // Calculate new parameters
            w_max_temp_ *= slowing_coeff_;
            stopping_angle_ = 5 * pow(w_max_temp_, 1.5) / 3 / sqrt(J_ROT_MAX_STOP_) * stopping_coeff_w_;

            reg_phase_ = 1;
            break;
        case 1:
            v_ref_ = 0;
            w_ref_ =
                synthesis_7(phi_error_, w_base_, alpha_, j_rot_max_temp_, stopping_angle_, w_max_temp_, W_MIN_, dt_);
            if (fabs(phi_error_) < PHI_TOL_)
            {
                reg_phase_ = 2;
                w_max_temp_ = W_MAX_;
            }
            break;
        case 2:
            distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
            distance_proj_ = distance_ * cos(phi_error_);

            starting_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(j_max_temp_) * starting_coeff_v_;
            stopping_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(J_MAX_STOP_) * stopping_coeff_v_;

            slowing_coeff_ =
                std::clamp(pow(distance_ / (starting_distance_ + stopping_distance_), 2.0 / 3.0), 0.0, 1.0);

            // Calculate new parameters
            v_max_temp_ *= slowing_coeff_;
            stopping_distance_ = 5 * pow(v_max_temp_, 1.5) / 3 / sqrt(J_MAX_STOP_) * stopping_coeff_v_;

            reg_phase_ = 3;
            break;
        case 3:
            distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
            distance_proj_ = distance_ * cos(phi_error_);

            v_ref_ = synthesis_7(distance_proj_ * direction_, v_base_, a_, j_max_temp_, stopping_distance_, v_max_temp_,
                                 V_MIN_, dt_);
            w_ref_ =
                P_w_ * std::clamp((distance_ - D_SHORT_TOL_) / (D_LONG_TOL_ - D_SHORT_TOL_), 0.0, 1.0) * phi_error_;

            if (distance_proj_ < D_PROJ_TOL_ * d_tol_perc_ && fabs(distance_) < D_TOL_ * d_tol_perc_)
            {
                reg_type_ = 0;
                reg_phase_ = 0;
                w_max_temp_ = W_MAX_;
                v_max_temp_ = V_MAX_;
                movement_state_ = -1;
            }

            break;
        }
    }

    void publish_motor_cmd()
    {
        auto msg = ros381_interfaces::msg::Float2();
        msg.float2[0] = v_right_;
        msg.float2[1] = v_left_;
        motor_cmd_publisher_->publish(msg);
    }

    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Current
        x_base_ = msg->pose.pose.position.x;
        y_base_ = msg->pose.pose.position.y;
        phi_base_ = tf2::getYaw(msg->pose.pose.orientation);
        v_base_ = msg->twist.twist.linear.x;
        w_base_ = msg->twist.twist.angular.z;
        time_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (!odom_initialized_)
            odom_initialized_ = true;
    }

    double synthesis_7(double distance, double velocity, double acceleration, double J_MAX, double stopping_distance,
                       double v_max, double v_min, double dt)
    {
        double abs_distance = fabs(distance);
        double abs_velocity = fabs(velocity);
        double abs_acceleration = fabs(acceleration);
        double v_ref = 0;
        if (dt <= 0 || std::isnan(dt))
            return 0.0;

        if (abs_distance <= stopping_distance)
        {
            double x = abs_distance / stopping_distance;
            v_ref = v_max * (35.0f * pow(x, 4) - 84.0f * pow(x, 5) + 70.0f * pow(x, 6) - 20.0f * pow(x, 7));
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
        v_ref = std::clamp(v_ref, v_min, v_max);

        return std::clamp(get_sign(distance) * v_ref, -v_max, v_max);
    }

    void declare_parameters()
    {
        this->declare_parameter("FREQ", 25.0);
        freq_ = this->get_parameter("FREQ").as_double();
        period_ = 1000000 / freq_;
        this->declare_parameter("L", 0.1545);
        L_ = this->get_parameter("L").as_double();
        this->declare_parameter("V_MAX", 1.0);
        V_MAX_ = this->get_parameter("V_MAX").as_double();
        this->declare_parameter("V_MIN", 0.1);
        V_MIN_ = this->get_parameter("V_MIN").as_double();
        this->declare_parameter("W_MAX", 6.28);
        W_MAX_ = this->get_parameter("W_MAX").as_double();
        this->declare_parameter("W_MIN", 0.126);
        W_MIN_ = this->get_parameter("W_MIN").as_double();
        this->declare_parameter("MOTOR_V_MAX", 1.0);
        MOTOR_V_MAX_ = this->get_parameter("MOTOR_V_MAX").as_double();
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
        this->declare_parameter("D_LONG_TOL", 0.1);
        D_LONG_TOL_ = this->get_parameter("D_LONG_TOL").as_double();
        this->declare_parameter("D_SHORT_TOL", 0.05);
        D_SHORT_TOL_ = this->get_parameter("D_SHORT_TOL").as_double();
        this->declare_parameter("PHI_TOL", 0.002);
        PHI_TOL_ = this->get_parameter("PHI_TOL").as_double();

        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  FREQ: %.2f", freq_);
        RCLCPP_INFO(this->get_logger(), "  L: %.4f", L_);
        RCLCPP_INFO(this->get_logger(), "  V_MAX: %.2f", V_MAX_);
        RCLCPP_INFO(this->get_logger(), "  V_MIN: %.2f", V_MIN_);
        RCLCPP_INFO(this->get_logger(), "  W_MAX: %.2f", W_MAX_);
        RCLCPP_INFO(this->get_logger(), "  W_MIN: %.3f", W_MIN_);
        RCLCPP_INFO(this->get_logger(), "  MOTOR_V_MAX: %.2f", MOTOR_V_MAX_);
        RCLCPP_INFO(this->get_logger(), "  P_w: %.2f", P_w_);
        RCLCPP_INFO(this->get_logger(), "  J_MAX: %.2f", J_MAX_);
        RCLCPP_INFO(this->get_logger(), "  J_MAX_STOP: %.2f", J_MAX_STOP_);
        RCLCPP_INFO(this->get_logger(), "  J_ROT_MAX: %.2f", J_ROT_MAX_);
        RCLCPP_INFO(this->get_logger(), "  J_ROT_MAX_STOP: %.2f", J_ROT_MAX_STOP_);
        RCLCPP_INFO(this->get_logger(), "  D_TOL: %.3f", D_TOL_);
        RCLCPP_INFO(this->get_logger(), "  D_PROJ_TOL: %.3f", D_PROJ_TOL_);
        RCLCPP_INFO(this->get_logger(), "  D_LONG_TOL: %.2f", D_LONG_TOL_);
        RCLCPP_INFO(this->get_logger(), "  D_SHORT_TOL: %.3f", D_SHORT_TOL_);
        RCLCPP_INFO(this->get_logger(), "  PHI_TOL: %.3f", PHI_TOL_);

        v_max_temp_ = V_MAX_;
        w_max_temp_ = W_MAX_;
        j_max_temp_ = J_MAX_;
        j_rot_max_temp_ = J_ROT_MAX_;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlLoopNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
