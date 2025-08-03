#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

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
        RCLCPP_INFO(this->get_logger(), "Global tactic node is running.");
    }

  private:
    unsigned long tick_period_;
    double time_ = 0; // [s]
    rclcpp::Time start_time_;
    bool match_started_ = false;
    bool chich_trigger_ = false, chich_waiting_ = true;

    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr chich_service_;

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

            // TODO: ovo izbaci odavde i napravi logiku u fsm-u za to, da bih mogao chich za vise stvari
            match_started_ = !match_started_;
            start_time_ = this->get_clock()->now();
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
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TacticGlobalNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}