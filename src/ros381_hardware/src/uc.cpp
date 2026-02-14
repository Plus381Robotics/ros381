#include "nav_msgs/msg/odometry.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "ros381_interfaces/srv/update_pose.hpp"
#include <atomic>
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
        if (!init_uart())
        {
            rclcpp::shutdown();
            return;
        }

        motor_cmd_sub_ = this->create_subscription<ros381_interfaces::msg::Float2>(
            "motor_cmd", 10, std::bind(&uCNode::motor_cmd_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        uart_rx_thread_ = std::thread(&uCNode::uart_rx_interrupt_loop, this);

        update_srv_ = this->create_service<ros381_interfaces::srv::UpdatePose>(
            "update_pose",
            std::bind(&uCNode::callback_update_pose, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "uC node running.");
    }

    ~uCNode()
    {
        running_ = false;
        std::terminate();
        if (uart_rx_thread_.joinable())
        {
            uart_rx_thread_.join();
        }
        if (uart_fd_ >= 0)
        {
            close(uart_fd_);
        }
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
                RCLCPP_INFO(this->get_logger(), "New pose :\nx = %.2f m\ny = %.2f m\nphi = %.2f rad",
                            x_base_, y_base_, phi_base_);

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
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t cmd_bytes[8];
        for (int i = 0; i < 8; i++)
            cmd_bytes[i] = 69;
        // double_to_bytes(msg->float2[0], &cmd_bytes[0]);
        // double_to_bytes(msg->float2[1], &cmd_bytes[3]);
        send_uart(cmd_bytes, 8);
        // RCLCPP_INFO(this->get_logger(), "Sent motor commands: %.3f, %.3f", msg->float2[0], msg->float2[1]);
    }

    bool init_uart()
    {
        uart_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY);
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
        read(uart_fd_, buffer, size);
    }

    void send_uart(const uint8_t *data, size_t size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        write(uart_fd_, data, size);
        tcdrain(uart_fd_);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uCNode>());
    rclcpp::shutdown();
    return 0;
}