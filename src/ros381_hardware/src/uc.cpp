#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "ros381_interfaces/msg/float3.hpp"

using namespace std::placeholders;

class uCNode : public rclcpp::Node
{
  public:
    uCNode() : Node("uc")
    {
        motor_cmd_sub_ = this->create_subscription<ros381_interfaces::msg::Float2>(
            "motor_cmd", 10, std::bind(&uCNode::callback_motor_cmd, this, _1));
        chinch_waiting_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "chinch_waiting", 10, std::bind(&uCNode::callback_chinch_waiting, this, _1));
        encoder_pub_ = this->create_publisher<ros381_interfaces::msg::Float3>("base_encoders", 10);
        switches_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("switches", 10);
        sensors_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("sensors", 10);
        chinch_trigger_pub_ = this->create_publisher<example_interfaces::msg::Bool>("chinch_trigger", 10);

        RCLCPP_INFO(this->get_logger(), "uC node is running.");
    }

  private:
    void uc_communication_loop()
    {
        // UART interrupt treba da triggeruje izvrsenje ove funkcije
        // uzme pristigle podatke
        memcpy(&from_stm[0], &v_passive_right_, sizeof(double));
        memcpy(&from_stm[1], &v_passive_left_, sizeof(double));
        memcpy(&from_stm[2], &dt_, sizeof(double));
        switches_ = (from_stm[3] >> (64 - 5)) & 0b11111;
        chinch_state_ = (from_stm[3] >> (64 - 5 - 1)) & 0b1;
        sensors_ = (from_stm[3] >> (64 - 5 - 1 - 8)) & 0b11111111;
        // pripremi poruku za stm
        memcpy(&v_motor_right_, &to_stm[0], sizeof(double));
        memcpy(&v_motor_left_, &to_stm[1], sizeof(double));
        memcpy(&chinch_waiting_, &to_stm[2], sizeof(int64_t));
        // TODO: posalje poruku stm-u
        // publish-uje
        publish_encoders();
        publish_chinch_trigger();
        publish_switches();
        publish_sensors();
		// podesi promenljive
		chinch_waiting_ = 0;
		chinch_prev_ = chinch_state_;
		// TODO: vidi za was promenljivu
    }

    void callback_motor_cmd(const ros381_interfaces::msg::Float2::SharedPtr msg)
    {
        v_motor_right_ = msg->float2[0];
        v_motor_left_ = msg->float2[1];
    }

    void callback_chinch_waiting(const example_interfaces::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        chinch_waiting_ = 1;
    }

    void publish_encoders()
    {
        auto msg = ros381_interfaces::msg::Float3();
        msg.float3[0] = v_passive_right_;
        msg.float3[1] = v_passive_left_;
        msg.float3[2] = dt_;
        encoder_pub_->publish(msg);
    }

    void publish_switches()
    {
        auto msg = example_interfaces::msg::UInt8();
        msg.data = switches_;
        switches_pub_->publish(msg);
    }

    void publish_sensors()
    {
        auto msg = example_interfaces::msg::UInt8();
        msg.data = sensors_;
        sensors_pub_->publish(msg);
    }

    void publish_chinch_trigger()
    {
        // TODO: porazmisli, (zapravo trigger) ili (ceka, a cinc nije tu, a cinc je bio tu u nekom trenutku)
        if ((chinch_state_ != chinch_prev_) || (chinch_state_ && chinch_waiting_ && chinch_was_))
        {
            auto msg = example_interfaces::msg::Bool();
            msg.data = chinch_state_;
            chinch_trigger_pub_->publish(msg);
        }
    }

    // void publish_hearbeat()
    // {
    //     auto msg = example_interfaces::msg::Empty();
    //     heartbeat_pub_->publish(msg);
    // }

    double v_motor_right_, v_motor_left_;
    double v_passive_right_, v_passive_left_, dt_;
    uint8_t switches_, sensors_;
    bool chinch_waiting_;
    bool chinch_state_, chinch_prev_, chinch_was_ = false;
    uint64_t from_stm[4], to_stm[3];

    rclcpp::Subscription<ros381_interfaces::msg::Float2>::SharedPtr motor_cmd_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr chinch_waiting_sub_;
    rclcpp::Publisher<ros381_interfaces::msg::Float3>::SharedPtr encoder_pub_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr switches_pub_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr sensors_pub_;
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr chinch_trigger_pub_;
    // rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr heartbeat_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uCNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
