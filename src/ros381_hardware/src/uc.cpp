#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include <atomic>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

class uCNode : public rclcpp::Node
{
  public:
    uCNode() : Node("uc")
    {
        uart_rx_thread_ = std::thread(&uCNode::uart_rx_interrupt_loop, this);
        uart2_rx_thread_ = std::thread(&uCNode::uart2_rx_loop, this);

        motor_cmd_sub_ = this->create_subscription<ros381_interfaces::msg::Float2>(
            "motor_cmd", 10, std::bind(&uCNode::motor_cmd_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        update_srv_ = this->create_service<ros381_interfaces::srv::UpdatePose>(
            "update_pose", std::bind(&uCNode::callback_update_pose, this, _1, _2));

        switches_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("switches", 10);
        vacuum_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
            "vacuum", 10, std::bind(&uCNode::vacuum_callback, this, _1));
        chinch_waiting_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "chinch_waiting", 10, std::bind(&uCNode::chinch_waiting_callback, this, _1));
        chinch_pub_ = this->create_publisher<example_interfaces::msg::Bool>("chinch_trigger", 10);

        if (!init_uart())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to init UART1.");
            rclcpp::shutdown();
            return;
        }
        if (!init_uart2())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to init UART2.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "uC node running.");
    }

    ~uCNode()
    {
        running_ = false;

        if (uart_rx_thread_.joinable())
        {
            uart_rx_thread_.join();
        }
        if (uart_fd_ >= 0)
        {
            close(uart_fd_);
        }
        uart2_running_ = false;

        if (uart2_rx_thread_.joinable())
        {
            uart2_rx_thread_.join();
        }

        if (uart2_fd_ >= 0)
        {
            close(uart2_fd_);
        }
        std::terminate();
    }

  private:
    void callback_update_pose(const std::shared_ptr<ros381_interfaces::srv::UpdatePose::Request> request,
                              std::shared_ptr<ros381_interfaces::srv::UpdatePose::Response> response)
    {

        response->success = false;

        bool update_x = (request->type / 100) % 10;
        bool update_y = (request->type / 10) % 10;
        bool update_phi = (request->type / 1) % 10;

        if (update_x)
        {
            req_x.store(request->x);
            set_x.store(true);
        }
        if (update_y)
        {
            req_y.store(request->y);
            set_y.store(true);
        }
        if (update_phi)
        {
            req_phi.store(request->phi);
            set_phi.store(true);
        }

        response->success = update_x || update_y || update_phi;
    }

    void uart_rx_interrupt_loop()
    {
        while (running_ && rclcpp::ok())
        {
            uint8_t raw_data[8];
            read_uart(raw_data, 8);
            for (uint8_t i = 0; i < 8; i++)
                process_rx_byte(raw_data[i]);

            double x_raw = (int16_t)(rxba[1] << 8 | rxba[0]) / 10000.0;
            double y_raw = (int16_t)(rxba[3] << 8 | rxba[2]) / 10000.0;
            double phi_raw = (int16_t)(rxba[5] << 8 | rxba[4]) / phi_conversion_;

            bool log = false;

            if (set_x.exchange(false))
            {
                x_base_offs_ = req_x.load() - x_base_;
                log = true;
            }
            if (set_y.exchange(false))
            {
                y_base_offs_ = req_y.load() - y_base_;
                log = true;
            }
            if (set_phi.exchange(false))
            {
                phi_base_offs_ = req_phi.load() - phi_base_;
                log = true;
            }

            x_base_ = x_raw + x_base_offs_;
            y_base_ = y_raw + y_base_offs_;
            phi_base_ = phi_raw + phi_base_offs_;

            if (log)
                RCLCPP_INFO(this->get_logger(), "New pose :\nx = %.2f m\ny = %.2f m\nphi = %.2f rad", x_base_, y_base_,
                            phi_base_);

            auto current_time = now();
            if (odom_initialized_)
            {
                double dt = (current_time - last_odom_time_).seconds();
                if (dt > 0.009 && dt < 0.011)
                {
                    dt = 0.01;
                    v_base_ = (x_base_ - prev_x_base_) / dt;
                    w_base_ = (phi_base_ - prev_phi_base_) / dt;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Bad dt: %.4f ms, setting velocities to 0.", dt * 1000);
                    v_base_ = 0;
                    w_base_ = 0;
                }
                publish_odometry();
            }
            else
            {
                odom_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Odometry initialized!");
            }
            last_odom_time_ = current_time;
            prev_x_base_ = x_base_;
            prev_y_base_ = y_base_;
            prev_phi_base_ = phi_base_;
        }
    }

    void process_rx_byte(uint8_t b)
    {
        sync = (sync << 8) | b;
        if (idx == 0)
        {
            if (sync == 0xFFFF)
                idx = 1;
            return;
        }
        rxba[idx - 1] = b;
        idx++;

        if (idx == 7)
            idx = 0;
    }

    void publish_odometry()
    {
        auto msg = nav_msgs::msg::Odometry();

        msg.header.stamp = now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = x_base_;
        msg.pose.pose.position.y = y_base_;
        msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, phi_base_);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.twist.twist.linear.x = v_base_;
        msg.twist.twist.linear.y = 0.0;
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = w_base_;
        odom_pub_->publish(msg);
    }

    void motor_cmd_callback(const ros381_interfaces::msg::Float2::SharedPtr msg)
    {
        uint8_t cmd_bytes[8];
        cmd_bytes[0] = 255;
        cmd_bytes[1] = 255;
        // for (int i = 2; i < 8; i++)
        //     cmd_bytes[i] = 69;
        int32_t cmdR_3B = motor_cmd_3B(msg->float2[0]);
        int32_t cmdL_3B = motor_cmd_3B(msg->float2[1]);
        cmd_bytes[2] = static_cast<uint8_t>((cmdR_3B >> 24) & 0xFF);
        cmd_bytes[3] = static_cast<uint8_t>((cmdR_3B >> 16) & 0xFF);
        cmd_bytes[4] = static_cast<uint8_t>((cmdR_3B >> 8) & 0xFF);
        cmd_bytes[5] = static_cast<uint8_t>((cmdL_3B >> 24) & 0xFF);
        cmd_bytes[6] = static_cast<uint8_t>((cmdL_3B >> 16) & 0xFF);
        cmd_bytes[7] = static_cast<uint8_t>((cmdL_3B >> 8) & 0xFF);
        send_uart(cmd_bytes, 8);
    }

    int32_t motor_cmd_3B(double cmd)
    {
        cmd = std::clamp(cmd, -4.0, 3.999);

        constexpr int SCALE = 1 << 21;
        int32_t cmd4B = static_cast<int32_t>(cmd * SCALE);
        int32_t cmd3B = cmd4B << 8;
        // RCLCPP_INFO(this->get_logger(), "cmd = %.4f, cmd4B = %d, cmd3B = %d", cmd, cmd4B, cmd3B);
        // RCLCPP_INFO(this->get_logger(), "cmd = %.4f, cmd4B = %s, cmd3B = %s", cmd,
        // std::bitset<32>(cmd4B).to_string().c_str(), std::bitset<32>(cmd3B).to_string().c_str());
        // RCLCPP_INFO(this->get_logger(), "Recovered cmd = %.4f", ((cmd3B >> 8) * pow(2, -21)));
        return cmd3B;
    }

    bool init_uart()
    {
        uart_fd_ = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
        if (uart_fd_ < 0)
            return false;

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(uart_fd_, &tty) != 0)
        {
            close(uart_fd_);
            return false;
        }

        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 8;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0)
        {
            close(uart_fd_);
            return false;
        }

        tcflush(uart_fd_, TCIOFLUSH);
        return true;
    }

    void read_uart(uint8_t *buffer, size_t size)
    {
        ssize_t n = read(uart_fd_, buffer, size);
        if (n < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "UART read failed");
        }
        else if (static_cast<size_t>(n) != size)
        {
            RCLCPP_INFO(this->get_logger(), "Recieved %ld bytes.", n);
            for (int i = 0; i < n; i++)
                RCLCPP_INFO(this->get_logger(), "%d", buffer[i]);
        }
    }

    void send_uart(const uint8_t *data, size_t size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        write(uart_fd_, data, size);
        tcdrain(uart_fd_);
    }

    bool init_uart2()
    {
        uart2_fd_ = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY);
        if (uart2_fd_ < 0)
            return false;

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(uart2_fd_, &tty) != 0)
        {
            close(uart2_fd_);
            return false;
        }

        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(uart2_fd_, TCSANOW, &tty) != 0)
        {
            close(uart2_fd_);
            return false;
        }

        tcflush(uart2_fd_, TCIOFLUSH);
        RCLCPP_INFO(this->get_logger(), "UART2 initialized!");
        return true;
    }

    void uart2_rx_loop()
    {
        while (uart2_running_ && rclcpp::ok())
        {
            uint8_t byte;
            int n = read(uart2_fd_, &byte, 1);
            if (n == 1)
            {
                uart2_rx_byte_.store(byte, std::memory_order_relaxed);

                chinch_ = byte & 0b1;
                auto chinch_msg = example_interfaces::msg::Bool();
                chinch_msg.data = chinch_;
                chinch_pub_->publish(chinch_msg);

                switches_ = ((byte >> 1) & 0b111) | (byte & (0b1 << 4));
                auto switches_msg = example_interfaces::msg::UInt8();
                switches_msg.data = switches_;
                switches_pub_->publish(switches_msg);

                uint8_t uart2_tx =
                    vacuum_.load(std::memory_order_relaxed) | chinch_waiting_.load(std::memory_order_relaxed);
                send_uart2_byte(uart2_tx);
            }
        }
    }

    void send_uart2_byte(uint8_t byte)
    {
        std::lock_guard<std::mutex> lock(uart2_mutex_);
        write(uart2_fd_, &byte, 1);
        tcdrain(uart2_fd_);
    }

    void vacuum_callback(const example_interfaces::msg::UInt8::SharedPtr msg)
    {
        vacuum_.store(msg->data & 0b00011110, std::memory_order_relaxed);
    }

    void chinch_waiting_callback(const example_interfaces::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        chinch_waiting_.store(0b1, std::memory_order_relaxed);
        if (clear_chinch_timer_)
            clear_chinch_timer_->cancel();
        clear_chinch_timer_ = this->create_wall_timer(1000ms, [this]() {
            chinch_waiting_.store(0b0, std::memory_order_relaxed);
            clear_chinch_timer_->cancel();
        });
    }

    uint8_t rxba[6];
    uint8_t idx = 0;
    uint16_t sync = 0;
    double phi_conversion_ = pow(2, 14) / M_PI;
    int uart_fd_ = -1;
    std::atomic<bool> running_{true};
    std::thread uart_rx_thread_;
    std::mutex mutex_;
    double x_base_ = 0.0, y_base_ = 0.0, phi_base_ = 0.0;
    double x_base_offs_ = 0.0, y_base_offs_ = 0.0, phi_base_offs_ = 0.0;
    std::atomic<bool> set_x{false}, set_y{false}, set_phi{false};
    std::atomic<double> req_x, req_y, req_phi;
    double v_base_ = 0.0, w_base_ = 0.0;
    double prev_x_base_ = 0.0, prev_y_base_ = 0.0, prev_phi_base_ = 0.0;
    bool odom_initialized_ = false;
    rclcpp::Time last_odom_time_;
    rclcpp::Subscription<ros381_interfaces::msg::Float2>::SharedPtr motor_cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<ros381_interfaces::srv::UpdatePose>::SharedPtr update_srv_;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr switches_pub_;
    rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr vacuum_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr chinch_waiting_sub_;
    rclcpp::TimerBase::SharedPtr clear_chinch_timer_;
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr chinch_pub_;

    int uart2_fd_ = -1;
    std::thread uart2_rx_thread_;
    std::atomic<bool> uart2_running_{true};

    std::atomic<uint8_t> vacuum_{0b0};
    std::atomic<uint8_t> chinch_waiting_{0b0};
    bool chinch_ = false;
    uint8_t switches_ = 0b0;
    std::atomic<uint8_t> uart2_rx_byte_{0};

    std::mutex uart2_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uCNode>());
    rclcpp::shutdown();
    return 0;
}
