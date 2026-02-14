#include "../include/signal.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "tf2/utils.h"
#include <example_interfaces/msg/int8.hpp>
#include <example_interfaces/msg/u_int8.hpp>
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
        // timer_ = this->create_wall_timer(std::chrono::microseconds(period_),
        //                                  std::bind(&ControlLoopNode::control_loop, this));
        motor_cmd_publisher_ = this->create_publisher<ros381_interfaces::msg::Float2>("motor_cmd", 10);
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ControlLoopNode::callback_odometry, this, _1));
        move_action_server_ =
            rclcpp_action::create_server<Move>(this, "move", std::bind(&ControlLoopNode::handle_goal, this, _1, _2),
                                               std::bind(&ControlLoopNode::handle_cancel, this, _1),
                                               std::bind(&ControlLoopNode::handle_accepted, this, _1));
        obstacle_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
            "obstacle_status", 10, std::bind(&ControlLoopNode::callback_obstacle, this, _1));

        obstacle_dir_pub_ = this->create_publisher<example_interfaces::msg::Int8>("obstacle_direction", 10);

        RCLCPP_INFO(this->get_logger(), "Control loop node is running.");
    }

  private:
    double L_, L_MIN_, L_MAX_; // [m]
    double eta_;
    double phi_base_, phi_error_, PHI_TOL_, phi_ref_ = 0.0;     // [rad]
    double x_base_, x_error_, x_ref_ = 0.0;                     // [m]
    double y_base_, y_error_, y_ref_ = 0.0;                     // [m]
    double v0_, v_base_, V_MAX_, V_MIN_, v_ref_ = 0.0, prev_v_; // [m/s]
    double MOTOR_V_MAX_;                                        // [m/s]
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
        J_ROT_MAX_STOP_;                          // [rad/s^3]
    unsigned long ctrl_time_ns_, ctrl_prev_time_; // [ns]
    unsigned long odom_time_ns_, odom_prev_time_; // [ns]
    double dt_;                                   // [s]
    double freq_;                                 // [Hz]
    short reg_type_ = 0, reg_phase_ = 0, movement_state_ = 0, direction_ = 1;
    unsigned long period_;                // [us]
    double v_right_ = 0.0, v_left_ = 0.0; // [m/s]
    bool odom_initialized_ = false;
    double V_SLOWED_MAX_ = 0.75; // [m/s] TODO: parametar
    unsigned stacked_cnt_ = 0;
    bool was_slowed_ = false;
    unsigned short obstacle_ = 0;
    bool obstacle_status_changed_ = false;
    int8_t obstacle_dir_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros381_interfaces::msg::Float2>::SharedPtr motor_cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp_action::Server<Move>::SharedPtr move_action_server_;
    std::shared_ptr<GoalHandleMove> current_goal_handle_;
    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr obstacle_dir_pub_;

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
        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            RCLCPP_INFO(get_logger(), "A goal is already active—rejecting new one.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        v_max_temp_ = std::clamp(goal->v_max, V_MIN_, V_MAX_);
        w_max_temp_ = std::clamp(goal->w_max, W_MIN_, W_MAX_);
        starting_coeff_v_ = std::clamp(goal->start_coeff_v, 1.0, 10.0);
        stopping_coeff_v_ = std::clamp(goal->stop_coeff_v, 1.0, 10.0);
        starting_coeff_w_ = std::clamp(goal->start_coeff_w, 1.0, 10.0);
        stopping_coeff_w_ = std::clamp(goal->stop_coeff_w, 1.0, 10.0);
        d_tol_perc_ = std::clamp(goal->distance_tolerance_percentage, 1.0, 10.0);
        phi_tol_perc_ = std::clamp(goal->angle_tolerance_percentage, 1.0, 10.0);
        direction_ = goal->direction;

        switch (goal->type)
        {
        // Rotate to Phi
        case -1:
            RCLCPP_INFO(this->get_logger(), "Rotate to PHI:\nphi = %.4f", goal->phi);
            x_ref_ = x_base_;
            y_ref_ = y_base_;
            phi_ref_ = goal->phi;
            reg_type_ = -1;
            break;
            // Rotate to XY
        case -2:
            x_ref_ = x_base_;
            y_ref_ = y_base_;
            phi_ref_ =
                wrap(atan2(goal->y - y_base_, goal->x - x_base_) + (goal->direction - 1) * M_PI * 0.5, -M_PI, M_PI);
            RCLCPP_INFO(this->get_logger(), "Rotate to XY:\nx = %.4f, y = %.4f, phi_ref = %.4f", goal->x, goal->y,
                        phi_ref_);
            reg_type_ = -1;
            break;
        // Move to XY
        case 1:
            x_ref_ = goal->x;
            y_ref_ = goal->y;
            phi_ref_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Move to XY:\nx = %.4f, y = %.4f", goal->x, goal->y);
            reg_type_ = 1;
            break;
            // Move on Direction
        case 2:
            x_ref_ = x_base_ + goal->direction * goal->y * cos(phi_base_);
            y_ref_ = y_base_ + goal->direction * goal->y * sin(phi_base_);
            phi_ref_ = phi_base_;
            RCLCPP_INFO(this->get_logger(), "Move on Direction:\ndistance = %.4f, direction = %d, phi = %.4f", goal->y,
                        goal->direction, phi_ref_);
            reg_type_ = 1;
            break;
        // Move on Direction Snapped
        case 3:
            x_ref_ = x_base_ + goal->direction * goal->y * cos(snap_angle(phi_base_, goal->phi));
            y_ref_ = y_base_ + goal->direction * goal->y * sin(snap_angle(phi_base_, goal->phi));
            phi_ref_ = snap_angle(phi_base_, goal->phi);
            RCLCPP_INFO(this->get_logger(), "Move on Direction Snapped:\ndistance = %.4f, direction = %d, phi = %.4f",
                        goal->y, goal->direction, phi_ref_);
            reg_type_ = 1;
            break;
        // Move on Angle
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
                movement_state_ = -2;

            feedback->angle_remaining = phi_error_;
            feedback->distance_remaininig = distance_proj_;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
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
                break;
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
    
    // TODO: delete
    int i = 0;
    void control_loop()
    {
        if (odom_initialized_)
        {
            ctrl_time_ns_ = now().nanoseconds();

            L_ = correct_param(L_, fabs(w_ref_) - fabs(w_base_), eta_, L_MIN_, L_MAX_);
            obstacle_dir_ = 0;
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

            v_right_ = v_ref_ + w_ref_ * L_ * 0.5;
            v_left_ = v_ref_ - w_ref_ * L_ * 0.5;
            double scale_factor = scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);
            v_ref_ *= scale_factor;
            w_ref_ *= scale_factor;

            dt_ = (ctrl_time_ns_ - ctrl_prev_time_) * 0.000000001;
            a_ = (v_base_ - prev_v_) / dt_;
            alpha_ = (w_base_ - prev_w_) / dt_;

            prev_v_ = v_base_;
            prev_w_ = w_base_;
            ctrl_prev_time_ = ctrl_time_ns_;

            this->publish_motor_cmd();
            this->publish_obstacle_dir();

            if (i < 100)
            {
                i++;
                RCLCPP_INFO(this->get_logger(), "odom time: %.3f us, ctrl time: %.3f us", odom_time_ns_ / 1000.0,
                                ctrl_time_ns_ / 1000.0);
                RCLCPP_INFO(this->get_logger(), "Difference: %.3f us", (-odom_time_ns_ + ctrl_time_ns_) / 1000.0);
            }
        }
    }

    void rotate()
    {
        phi_error_ = wrap(phi_ref_ - phi_base_, -M_PI, M_PI);

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
        v_ref_ = 0;
        w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_, j_rot_max_temp_, stopping_angle_, w_max_temp_, W_MIN_,
                                    dt_, 0.0, 0, 0.0);
        if (fabs(phi_error_) < PHI_TOL_ * phi_tol_perc_)
        {
            reset_movement();
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
            w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_, j_rot_max_temp_, stopping_angle_, w_max_temp_,
                                        W_MIN_, dt_, 0.0, 0, 0.0);
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
            if (obstacle_status_changed_)
            {
                v0_ = v_base_;
                obstacle_status_changed_ = false;
            }
            v_ref_ = velocity_synthesis(distance_proj_ * direction_, v_base_, a_, j_max_temp_, stopping_distance_,
                                        v_max_temp_, V_MIN_, dt_, v0_, obstacle_, V_SLOWED_MAX_);
            w_ref_ =
                P_w_ * std::clamp((distance_ - D_SHORT_TOL_) / (D_LONG_TOL_ - D_SHORT_TOL_), 0.0, 1.0) * phi_error_;

            obstacle_dir_ = get_sign(v_ref_);
            if (distance_proj_ < D_PROJ_TOL_ * d_tol_perc_ && fabs(distance_) < D_TOL_ * d_tol_perc_)
            {
                reset_movement();
                movement_state_ = -1;
            }
            else if (obstacle_ == 1 && fabs(v_base_) < V_MIN_ && fabs(w_base_) < W_MIN_)
            {
                reset_movement();
                movement_state_ = -4;
            }
            else if (stacked(0.5, v_base_, V_MIN_, freq_, &stacked_cnt_))
            {
                reset_movement();
                movement_state_ = -3;
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

    void publish_obstacle_dir()
    {
        auto msg = example_interfaces::msg::Int8();
        msg.data = obstacle_dir_;
        obstacle_dir_pub_->publish(msg);
    }

    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_base_ = msg->pose.pose.position.x;
        y_base_ = msg->pose.pose.position.y;
        phi_base_ = tf2::getYaw(msg->pose.pose.orientation);
        v_base_ = msg->twist.twist.linear.x;
        w_base_ = msg->twist.twist.angular.z;
        // odom_time_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        odom_time_ns_ = now().nanoseconds();
        if (!odom_initialized_)
        {
            odom_initialized_ = true;
            odom_prev_time_ = odom_time_ns_;
            prev_v_ = v_base_;
            prev_w_ = w_base_;
            // }

            // if (!timer_)
            // {
            // Small delay to ensure odom just arrived
            rclcpp::sleep_for(std::chrono::microseconds(750));
            ctrl_prev_time_ = now().nanoseconds();
            control_loop();

            timer_ = this->create_wall_timer(std::chrono::microseconds(period_),
                                             std::bind(&ControlLoopNode::control_loop, this));

            RCLCPP_INFO(this->get_logger(), "Control timer started with 750µs delay after first odom");
        }
    }

    void reset_movement()
    {
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
        reg_type_ = 0;
        reg_phase_ = 0;
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
        this->declare_parameter("L_MIN", 0.1055);
        L_MIN_ = this->get_parameter("L_MIN").as_double();
        this->declare_parameter("L_MAX", 0.2035);
        L_MAX_ = this->get_parameter("L_MAX").as_double();
        this->declare_parameter("eta", 0.0);
        eta_ = this->get_parameter("eta").as_double();

        // RCLCPP_INFO(this->get_logger(), "Parameters:");
        // RCLCPP_INFO(this->get_logger(), "  FREQ: %.2f", freq_);
        // RCLCPP_INFO(this->get_logger(), "  L: %.4f", L_);
        // RCLCPP_INFO(this->get_logger(), "  V_MAX: %.2f", V_MAX_);
        // RCLCPP_INFO(this->get_logger(), "  V_MIN: %.2f", V_MIN_);
        // RCLCPP_INFO(this->get_logger(), "  W_MAX: %.2f", W_MAX_);
        // RCLCPP_INFO(this->get_logger(), "  W_MIN: %.3f", W_MIN_);
        // RCLCPP_INFO(this->get_logger(), "  MOTOR_V_MAX: %.2f", MOTOR_V_MAX_);
        // RCLCPP_INFO(this->get_logger(), "  P_w: %.2f", P_w_);
        // RCLCPP_INFO(this->get_logger(), "  J_MAX: %.2f", J_MAX_);
        // RCLCPP_INFO(this->get_logger(), "  J_MAX_STOP: %.2f", J_MAX_STOP_);
        // RCLCPP_INFO(this->get_logger(), "  J_ROT_MAX: %.2f", J_ROT_MAX_);
        // RCLCPP_INFO(this->get_logger(), "  J_ROT_MAX_STOP: %.2f", J_ROT_MAX_STOP_);
        // RCLCPP_INFO(this->get_logger(), "  D_TOL: %.3f", D_TOL_);
        // RCLCPP_INFO(this->get_logger(), "  D_PROJ_TOL: %.3f", D_PROJ_TOL_);
        // RCLCPP_INFO(this->get_logger(), "  D_LONG_TOL: %.2f", D_LONG_TOL_);
        // RCLCPP_INFO(this->get_logger(), "  D_SHORT_TOL: %.3f", D_SHORT_TOL_);
        // RCLCPP_INFO(this->get_logger(), "  PHI_TOL: %.3f", PHI_TOL_);

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
