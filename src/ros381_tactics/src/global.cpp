#include "ros381_tactics/global.hpp"

TacticGlobalNode::TacticGlobalNode() : Node("tactic_global"), guard_{}
{
    guard_ = std::make_unique<py::scoped_interpreter>();

    this->declare_parameter("tick_freq", 50.0);
    tick_period_ = 1000 / this->get_parameter("tick_freq").as_double();
    this->declare_parameter("default_tactic", 0);
    tactic_num_ = this->get_parameter("default_tactic").as_int();
    this->declare_parameter("default_side", "yellow");
    auto side_str = this->get_parameter("default_side").as_string();
    if (side_str == "yellow")
        tactic_side_ = -1;
    else
        tactic_side_ = 1;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_period_),
                                     std::bind(&TacticGlobalNode::tactic_tick, this));
    time_pub_ = this->create_publisher<example_interfaces::msg::Float32>("tactic_time", 10);
    // chinch_service_ = this->create_service<example_interfaces::srv::Trigger>(
    //     "chinch_service", std::bind(&TacticGlobalNode::chinch_trigger, this, _1, _2));
    pose_client_ = this->create_client<ros381_interfaces::srv::UpdatePose>("update_pose");
    move_client_ = rclcpp_action::create_client<Move>(this, "move");
    chinch_waiting_pub_ = this->create_publisher<example_interfaces::msg::Empty>("chinch_waiting", 10);
    chinch_trigger_sub_ = this->create_subscription<example_interfaces::msg::Bool>(
        "chinch_trigger", 10, std::bind(&TacticGlobalNode::callback_chinch_state, this, _1));
    switches_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
        "switches", 10, std::bind(&TacticGlobalNode::callback_switches, this, _1));
    pose_offs_pub_ = this->create_publisher<ros381_interfaces::msg::Float3>("pose_offset", 10);
    ax_move_client_ = rclcpp_action::create_client<AxMove>(this, "ax_move");
    ax_bulk_move_client_ = rclcpp_action::create_client<AxBulkMove>(this, "ax_bulk_move");
    ax_hybrid_move_client_ = rclcpp_action::create_client<AxHybridMove>(this, "ax_hybrid_move");

    init_python(this);

    RCLCPP_INFO(this->get_logger(), "Global tactic node is running.");
}

void TacticGlobalNode::publish_pose_offset(double x, double y, double phi)
{
    auto msg = ros381_interfaces::msg::Float3();
    msg.float3[0] = x;
    msg.float3[1] = y;
    msg.float3[2] = phi;
    pose_offs_pub_->publish(msg);
}

void TacticGlobalNode::callback_switches(const example_interfaces::msg::UInt8::SharedPtr msg)
{
    reset_on_ = (bool)((msg->data >> 4) & 0b1);
    tactic_side_ = ((msg->data >> 3) & 0b1) ? 1 : -1;
    tactic_num_ = msg->data & 0b111;
}

void TacticGlobalNode::callback_chinch_state(const example_interfaces::msg::Bool::SharedPtr msg)
{
    if (msg->data) // rastuca ivica
    {
        if (chinch_waiting_)
        {
            chinch_trigger_ = true;
        }
    }
    else // opadajuca ivica
    {
    }
}

void TacticGlobalNode::pub_chinch_waiting()
{
    if (chinch_waiting_)
    {
        auto msg = example_interfaces::msg::Empty();
        chinch_waiting_pub_->publish(msg);
    }
}

void TacticGlobalNode::ax_bulk_move_goal(const std::vector<AxMoveGoal> &goals)
{
    ax_bulk_move_result_ = 0;
    rclcpp::Rate rate(std::chrono::milliseconds(100));

    auto start = this->now();
    while (rclcpp::ok() && (this->now() - start) < rclcpp::Duration::from_seconds(0.5))
    {
        if (this->ax_bulk_move_client_->wait_for_action_server())
        {
            break;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
        rate.sleep();
    }
    if (!this->ax_bulk_move_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
        return;
    }

    auto goal_msg = AxBulkMove::Goal();
    goal_msg.id.resize(goals.size());
    goal_msg.position.resize(goals.size());
    goal_msg.velocity.resize(goals.size());
    goal_msg.position_tolerance.resize(goals.size());
    uint8_t cnt = 0;
    for (auto &goal : goals)
    {
        goal_msg.id[cnt] = goal.id;
        goal_msg.position[cnt] = goal.position;
        goal_msg.velocity[cnt] = goal.velocity;
        goal_msg.position_tolerance[cnt] = goal.position_tolerance;
        cnt++;
    }

    RCLCPP_INFO(this->get_logger(), "Sending ax bulk move goal...");

    auto send_goal_options = rclcpp_action::Client<AxBulkMove>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TacticGlobalNode::ax_bulk_move_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TacticGlobalNode::ax_bulk_move_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TacticGlobalNode::ax_bulk_move_result_callback, this, _1);
    this->ax_bulk_move_client_->async_send_goal(goal_msg, send_goal_options);
}

void TacticGlobalNode::ax_bulk_move_goal_response_callback(const AxBulkMoveGoalHandle::SharedPtr &goal_handle)
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

void TacticGlobalNode::ax_bulk_move_feedback_callback(AxBulkMoveGoalHandle::SharedPtr,
                                                      const std::shared_ptr<const AxBulkMove::Feedback> feedback)
{
    for (size_t i = 0; i < feedback->current_position.size(); ++i)
    {
        RCLCPP_DEBUG(this->get_logger(), "  Motor %zu: Pos=%d, Err=%d, Vel=%d", i, feedback->current_position[i],
                     feedback->position_error[i], feedback->current_velocity[i]);
    }
}

void TacticGlobalNode::ax_bulk_move_result_callback(const AxBulkMoveGoalHandle::WrappedResult &result)
{
    ax_bulk_move_result_ = result.result->status;
    RCLCPP_INFO(this->get_logger(), "Move status: %d", ax_bulk_move_result_);
}

void TacticGlobalNode::ax_move_goal(AxMoveGoal goal)
{
    ax_move_result_ = 0;
    rclcpp::Rate rate(std::chrono::milliseconds(100));

    auto start = this->now();
    while (rclcpp::ok() && (this->now() - start) < rclcpp::Duration::from_seconds(0.5))
    {
        if (this->ax_move_client_->wait_for_action_server())
        {
            break;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
        rate.sleep();
    }
    if (!this->ax_move_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
        return;
    }

    auto goal_msg = AxMove::Goal();
    goal_msg.id = goal.id;
    goal_msg.position = goal.position;
    goal_msg.velocity = goal.velocity;
    goal_msg.position_tolerance = goal.position_tolerance;

    RCLCPP_INFO(this->get_logger(), "Sending ax move goal...");

    auto send_goal_options = rclcpp_action::Client<AxMove>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TacticGlobalNode::ax_move_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TacticGlobalNode::ax_move_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TacticGlobalNode::ax_move_result_callback, this, _1);
    this->ax_move_client_->async_send_goal(goal_msg, send_goal_options);
}

void TacticGlobalNode::ax_move_goal_response_callback(const AxMoveGoalHandle::SharedPtr &goal_handle)
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

void TacticGlobalNode::ax_move_feedback_callback(AxMoveGoalHandle::SharedPtr,
                                                 const std::shared_ptr<const AxMove::Feedback> feedback)
{
    RCLCPP_DEBUG(this->get_logger(), "Current position: %d\t Position error: %d\t Current velocity: %d",
                 feedback->current_position, feedback->position_error, feedback->current_velocity);
}

void TacticGlobalNode::ax_move_result_callback(const AxMoveGoalHandle::WrappedResult &result)
{
    ax_move_result_ = result.result->status;
    RCLCPP_INFO(this->get_logger(), "Move status: %d", ax_move_result_);
}

void TacticGlobalNode::ax_hybrid_move_goal(uint8_t id, uint16_t velocity, float zero_time, int16_t delta_pos)
{
    ax_hybrid_move_result_ = 0;
    rclcpp::Rate rate(std::chrono::milliseconds(100));

    auto start = this->now();
    while (rclcpp::ok() && (this->now() - start) < rclcpp::Duration::from_seconds(0.5))
    {
        if (this->ax_hybrid_move_client_->wait_for_action_server())
        {
            break;
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
        rate.sleep();
    }
    if (!this->ax_hybrid_move_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
        return;
    }

    auto goal_msg = AxHybridMove::Goal();
    goal_msg.id = id;
    goal_msg.velocity = velocity;
    goal_msg.zero_time = zero_time;
    goal_msg.delta_pos = delta_pos;

    RCLCPP_INFO(this->get_logger(), "Sending ax hybrid move goal...");

    auto send_goal_options = rclcpp_action::Client<AxHybridMove>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TacticGlobalNode::ax_hybrid_move_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TacticGlobalNode::ax_hybrid_move_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TacticGlobalNode::ax_hybrid_move_result_callback, this, _1);
    this->ax_hybrid_move_client_->async_send_goal(goal_msg, send_goal_options);
}

void TacticGlobalNode::ax_hybrid_move_goal_response_callback(const AxHybridMoveGoalHandle::SharedPtr &goal_handle)
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

void TacticGlobalNode::ax_hybrid_move_feedback_callback(AxHybridMoveGoalHandle::SharedPtr,
                                                        const std::shared_ptr<const AxHybridMove::Feedback> feedback)
{
    RCLCPP_DEBUG(this->get_logger(), "Current position: %d\tCurrent velocity: %d", feedback->current_position,
                 feedback->current_velocity);
}

void TacticGlobalNode::ax_hybrid_move_result_callback(const AxHybridMoveGoalHandle::WrappedResult &result)
{
    ax_hybrid_move_result_ = result.result->status;
    ax_hybrid_end_position_ = result.result->position;
    RCLCPP_INFO(this->get_logger(), "Move status: %d, Position: %d", ax_hybrid_move_result_, ax_hybrid_end_position_);
}

void TacticGlobalNode::send_goal(int type, double x, double y, double phi, int8_t direction, double v_max, double w_max,
                                 double distance_tolerance_percentage, double angle_tolerance_percentage,
                                 double start_coeff_v, double start_coeff_w, double stop_coeff_v, double stop_coeff_w)
{
    move_result_ = 0;
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

    RCLCPP_INFO(this->get_logger(), "Sending movement goal...");

    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TacticGlobalNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TacticGlobalNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TacticGlobalNode::result_callback, this, _1);
    this->move_client_->async_send_goal(goal_msg, send_goal_options);
}

void TacticGlobalNode::cancel_goal()
{
    RCLCPP_INFO(this->get_logger(), "Cancelling movement goal...");
    move_client_->async_cancel_all_goals();
}

void TacticGlobalNode::global_fsm()
{
    pub_chinch_waiting();
    global_state_ = reset_on_ ? 0 : global_state_;
    switch (global_state_)
    {
    case 0:
        if (reset_on_)
        {
            if (!reset_was_on_)
            {
                RCLCPP_INFO(this->get_logger(), "Reset is on...");
                chinch_waiting_ = false;
                tactic_result_ = tactics_module_->attr("reset_tactic")();
                py::module::import("sys").attr("stdout").attr("flush")();
                reset_was_on_ = true;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Going to GL_CHICH_1");
            chinch_waiting_ = true;
            reset_was_on_ = false;
            global_state_ = GL_CHICH_1;
        }
        break;
    case GL_CHICH_1:
        if (chinch_trigger_)
        {
            global_state_ = GL_LOAD_TACTIC;
            chinch_waiting_ = false;
            chinch_trigger_ = false;
            RCLCPP_INFO(this->get_logger(), "Going to GL_LOAD_TACTIC");
        }
        break;
    case GL_LOAD_TACTIC:
        tactic_result_ = tactics_module_->attr("load_tactic")(this, tactic_num_, tactic_side_);
        py::module::import("sys").attr("stdout").attr("flush")();
        if (tactic_result_.cast<int>() == -1)
        {
            global_state_ = GL_CHICH_2;
            chinch_waiting_ = true;
            RCLCPP_INFO(this->get_logger(), "Going to GL_CHICH_2");
        }
        break;
    case GL_CHICH_2:
        if (chinch_trigger_)
        {
            global_state_ = GL_TACTIC;
            chinch_waiting_ = false;
            chinch_trigger_ = false;
            match_started_ = true;
            start_time_ = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(), "Going to GL_TACTIC");
        }
        break;
    case GL_TACTIC:
        tactic_result_ = tactics_module_->attr("execute_tactic")();
        py::module::import("sys").attr("stdout").attr("flush")();
        if (tactic_result_.cast<int>() == -1)
        {
            RCLCPP_INFO(this->get_logger(), "Tactic completed successfully");
            global_state_ = GL_END;
        }
        break;
    case GL_END:
        RCLCPP_INFO(this->get_logger(), "Match ended.");
        RCLCPP_INFO(this->get_logger(), "Time: %.3f", time_);
        rclcpp::shutdown();
        break;
    }
}

void TacticGlobalNode::goal_response_callback(const GoalHandleMove::SharedPtr &goal_handle)
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

void TacticGlobalNode::feedback_callback(GoalHandleMove::SharedPtr,
                                         const std::shared_ptr<const Move::Feedback> feedback)
{
    RCLCPP_DEBUG(this->get_logger(), "Remaining distance: %.2f; Remaining angle: %.2f", feedback->distance_remaininig,
                 feedback->angle_remaining);
}

void TacticGlobalNode::result_callback(const GoalHandleMove::WrappedResult &result)
{
    move_result_ = result.result->status;
    RCLCPP_INFO(this->get_logger(), "Move status: %d", move_result_);
}

void TacticGlobalNode::pub_time()
{
    auto msg = example_interfaces::msg::Float32();
    msg.data = time_;
    time_pub_->publish(msg);
}

void TacticGlobalNode::tactic_tick()
{
    if (match_started_)
    {
        time_ = (this->get_clock()->now().nanoseconds() - start_time_.nanoseconds()) / 1000000000.0;
        pub_time();
    }
    global_fsm();
    if (time_ > 100.0 && match_started_)
    {
        global_state_ = GL_END;
        RCLCPP_INFO(this->get_logger(), "Time ran out.");
        match_started_ = false;
    }
}

void TacticGlobalNode::update_pose(double x, double y, double phi, uint16_t type)
{
    update_pose_result_ = 0;
    while (!pose_client_->wait_for_service(std::chrono::milliseconds(50)))
    {
        if (rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Terminating...");
            update_pose_result_ = -2;
            return;
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
}

void TacticGlobalNode::update_pose_callback(rclcpp::Client<ros381_interfaces::srv::UpdatePose>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Pose updated");
    update_pose_result_ = -1;
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