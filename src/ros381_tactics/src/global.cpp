#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros381_interfaces/action/move.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include "ros381_tactics/defines.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pybind11/embed.h>

using namespace std::placeholders;
using Move = ros381_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

namespace py = pybind11;

class TacticGlobalNode : public rclcpp::Node
{
  public:
    TacticGlobalNode() : Node("tactic_global")
    {
        this->declare_parameter("tick_freq", 50.0);
        tick_period_ = 1000 / this->get_parameter("tick_freq").as_double();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_period_),
                                         std::bind(&TacticGlobalNode::tactic_tick, this));
        time_pub_ = this->create_publisher<example_interfaces::msg::Float32>("tactic_time", 10);
        chich_service_ = this->create_service<example_interfaces::srv::Trigger>(
            "chich_service", std::bind(&TacticGlobalNode::chich_trigger, this, _1, _2));
        pose_client_ = this->create_client<ros381_interfaces::srv::UpdatePose>("update_pose");
        move_client_ = rclcpp_action::create_client<Move>(this, "move");

        try
        {
            std::string install_path = ament_index_cpp::get_package_share_directory("ros381_tactics");
            py::module sys = py::module::import("sys");
            sys.attr("path").attr("append")(install_path + "/../lib/python3.10/site-packages");
            tactics_module_ = py::module::import("ros381_tactics.individual_tactics");
            tactics_module_.attr("hello_tactics")();
            py::module::import("sys").attr("stdout").attr("flush")();
        }
        catch (const py::error_already_set &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Python init failed: %s", e.what());
            rclcpp::shutdown();
        }
        try
        {
            // Import the embedded module
            py::module embedded = py::module::import("ros381_tactics_py");

            // Pass 'this' to Python
            tactics_module_.attr("node_instance") = embedded.attr("TacticGlobalNode")(this);

            RCLCPP_INFO(this->get_logger(), "C++ methods exposed to Python");
        }
        catch (const py::error_already_set &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to expose C++ methods: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Global tactic node is running.");
    }

    void send_goal(int type, double x, double y, double phi, int8_t direction, double v_max, double w_max,
                   double distance_tolerance_percentage, double angle_tolerance_percentage, double start_coeff_v,
                   double start_coeff_w, double stop_coeff_v, double stop_coeff_w)
    {
        rclcpp::Rate rate(std::chrono::milliseconds(100));

        auto start = this->now();
        while (rclcpp::ok() && (this->now() - start) < rclcpp::Duration::from_seconds(0.5))
        {
            if (this->move_client_->wait_for_action_server())
            {
                break;
            }
            RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
            rate.sleep();
        }
        if (!this->move_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
            return;
        }

        auto goal_msg = Move::Goal();
        goal_msg.type = type;
        goal_msg.x = x;
        goal_msg.y = y;
        goal_msg.phi = phi;
        goal_msg.direction = direction;
        goal_msg.v_max = v_max;
        goal_msg.w_max = w_max;
        goal_msg.distance_tolerance_percentage = distance_tolerance_percentage;
        goal_msg.angle_tolerance_percentage = angle_tolerance_percentage;
        goal_msg.start_coeff_v = start_coeff_v;
        goal_msg.start_coeff_w = start_coeff_w;
        goal_msg.stop_coeff_v = stop_coeff_v;
        goal_msg.stop_coeff_w = stop_coeff_w;

        RCLCPP_INFO(this->get_logger(), "Sending movement goal.");

        auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TacticGlobalNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&TacticGlobalNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TacticGlobalNode::result_callback, this, _1);
        this->move_client_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    unsigned long tick_period_;
    double time_ = 0; // [s]
    rclcpp::Time start_time_;
    bool match_started_ = false;
    bool chich_trigger_ = false, chich_waiting_ = true;
    int8_t global_state_ = 0;
    int8_t move_result_ = 0;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr chich_service_;
    rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedPtr pose_client_;
    rclcpp_action::Client<Move>::SharedPtr move_client_;

    py::scoped_interpreter guard_{};
    py::module tactics_module_;

    void global_fsm()
    {
        switch (global_state_)
        {
        case 0:
            RCLCPP_INFO(this->get_logger(), "Initial state... Going to GL_CHICH_1");
            global_state_ = GL_CHICH_1;
            break;
        case GL_CHICH_1:
            if (chich_trigger_)
            {
                global_state_ = GL_CALIBRATION;
                chich_waiting_ = false;
                chich_trigger_ = false;
                RCLCPP_INFO(this->get_logger(), "Going to GL_CALIBRATION");
            }
            break;
        case GL_CALIBRATION:
            global_state_ = GL_CHICH_2;
            chich_waiting_ = true;
            RCLCPP_INFO(this->get_logger(), "Going to GL_CHICH_2");
            break;
        case GL_CHICH_2:
            if (chich_trigger_)
            {
                global_state_ = GL_TACTIC;
                chich_waiting_ = false;
                chich_trigger_ = false;
                match_started_ = !match_started_;
                start_time_ = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "Going to GL_TACTIC");
            }
            break;
        case GL_TACTIC:
            tactics_module_.attr("tactic_0")(this);
            py::module::import("sys").attr("stdout").attr("flush")();
            global_state_ = -1;
            break;
        case -1:
            if (move_result_ == -1)
            {
                RCLCPP_INFO(this->get_logger(), "Move finished.");
                global_state_ = GL_END;
            }
            break;
        case GL_END:
            RCLCPP_INFO(this->get_logger(), "Tactic ended.");
            rclcpp::shutdown();
            break;
        }
    }

    void goal_response_callback(const GoalHandleMove::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result.");
        }
    }

    void feedback_callback(GoalHandleMove::SharedPtr, const std::shared_ptr<const Move::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Remaining distance: %.2f; Remaining angle: %.2f",
                    feedback->distance_remaininig, feedback->angle_remaining);
    }

    void result_callback(const GoalHandleMove::WrappedResult &result)
    {
        move_result_ = result.result->status;
        RCLCPP_INFO(this->get_logger(), "Move status: %d", move_result_);
    }

    void pub_time()
    {
        auto msg = example_interfaces::msg::Float32();
        msg.data = time_;
        time_pub_->publish(msg);
    }

    void chich_trigger(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
                       std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        (void)request;
        if (chich_waiting_)
        {
            chich_trigger_ = true;
            response->success = true;
            response->message = "Triggered chich";
        }
        else
        {
            response->success = false;
            response->message = "Chich was not waiting for trigger";
        }
    }

    void tactic_tick()
    {
        if (match_started_)
        {
            time_ = (this->get_clock()->now().nanoseconds() - start_time_.nanoseconds()) / 1000000000.0;
            pub_time();
        }
        global_fsm();
    }

    int update_pose(double x, double y, double phi, uint16_t type)
    {
        while (!pose_client_->wait_for_service(std::chrono::milliseconds(50)))
        {
            if (rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Terminating...");
                return 1;
            }
            RCLCPP_INFO(this->get_logger(), "Service Unavailable. Waiting for Service...");
        }

        auto request = std::make_shared<ros381_interfaces::srv::UpdatePose::Request>();
        bool update_x = (type / 100) % 10;
        bool update_y = (type / 10) % 10;
        bool update_phi = (type / 1) % 10;

        request->type = type;
        if (update_x)
            request->x = x;
        if (update_y)
            request->y = y;
        if (update_phi)
            request->phi = phi;

        auto result_future = pose_client_->async_send_request(
            request, std::bind(&TacticGlobalNode::update_pose_callback, this, std::placeholders::_1));
        return -1;
    }

    void update_pose_callback(rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedFuture future)
    {
        auto status = future.wait_for(std::chrono::milliseconds(50));
        if (status == std::future_status::ready)
            RCLCPP_INFO(this->get_logger(), "Pose updated");
        else
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
};

PYBIND11_EMBEDDED_MODULE(ros381_tactics_py, m)
{
    py::class_<TacticGlobalNode, std::shared_ptr<TacticGlobalNode>>(m, "TacticGlobalNode")
        .def("send_goal", &TacticGlobalNode::send_goal);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TacticGlobalNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}