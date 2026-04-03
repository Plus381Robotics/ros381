#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros381_interfaces/msg/float2.hpp"
#include "ros381_interfaces/msg/mini_mbp.hpp"
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

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        update_srv_ = this->create_service<ros381_interfaces::srv::UpdatePose>(
            "update_pose", std::bind(&uCNode::callback_update_pose, this, _1, _2));

        switches_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("switches", 10);
        vacuum_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
            "vacuum", 10, std::bind(&uCNode::vacuum_callback, this, _1));
        chinch_waiting_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "chinch_waiting", 10, std::bind(&uCNode::chinch_waiting_callback, this, _1));
        chinch_pub_ = this->create_publisher<example_interfaces::msg::Bool>("chinch_trigger", 10);

        mini_mbp_sub_ = this->create_subscription<ros381_interfaces::msg::MiniMBP>(
            "mini_mbp", 10, std::bind(&uCNode::minimbp_callback, this, _1));
        move_status_budz_pub_ = this->create_publisher<example_interfaces::msg::Int8>("move_status_budz", 10);

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
    void minimbp_callback(const ros381_interfaces::msg::MiniMBP::SharedPtr msg)
    {
        mbp_type = msg->type;
        mbp_x = msg->x;
        mbp_y = msg->y;
        mbp_phi = msg->phi;
        mbp_direction = msg->direction;
        mbp_obstacle = msg->obstacle;
        mbp_v_max_100 = msg->v_max_100;
        mbp_w_max_10 = msg->w_max_10;
        mbp_tol_perc = msg->tol_perc;
        mbp_coeff = msg->coeff;

        send_mbp_packet();
    }

    void send_mbp_packet()
    {
        uint8_t tx[40] = {0};

        // First 8 bytes all 0xFF
        for (int i = 0; i < 8; i++)
            tx[i] = 0xFF;

        int base = 8;

        tx[base + 0] = mbp_type;

        memcpy(tx + base + 1, &mbp_x, sizeof(double));
        memcpy(tx + base + 1 + 8, &mbp_y, sizeof(double));
        memcpy(tx + base + 1 + 16, &mbp_phi, sizeof(double));

        uint8_t dir_bits = (mbp_direction == -1) ? 0b10 : 0b01;
        tx[base + 25] = (dir_bits << 6) | (mbp_obstacle & 0b111111);

        tx[base + 26] = mbp_v_max_100; // 0..100
        tx[base + 27] = mbp_w_max_10;  // 0..10

        uint8_t tol_byte = (((mbp_tol_perc >> 4) & 0b1111) << 4) | (mbp_tol_perc & 0b1111);
        tx[base + 28] = tol_byte;

        uint8_t coeff_byte =
            (((mbp_coeff & 0b11) << 6) | ((mbp_coeff & 0b11) << 4) | ((mbp_coeff & 0b11) << 2) | (mbp_coeff & 0b11));
        tx[base + 29] = coeff_byte;

        // checksum treba od 8
        uint16_t checksum = fletcher16(tx, 30);
        tx[base + 30] = checksum & 0xFF;
        tx[base + 31] = (checksum >> 8) & 0xFF;

        send_uart(tx, 40);
    }

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
            RCLCPP_INFO(this->get_logger(), "Updating x to = %.3f", request->x);
        }
        if (update_y)
        {
            req_y.store(request->y);
            set_y.store(true);
            RCLCPP_INFO(this->get_logger(), "Updating y to = %.3f", request->y);
        }
        if (update_phi)
        {
            req_phi.store(request->phi);
            set_phi.store(true);
            RCLCPP_INFO(this->get_logger(), "Updating phi to = %.3f", request->phi);
        }

        response->success = update_x || update_y || update_phi;
    }

    uint8_t create_rxba()
    {
        int start_index = -1;

        for (int i = 0; i <= 40 - 8; i++)
        {
            if (rx_buffer[i] == 0xFF && rx_buffer[i + 1] == 0xFF && rx_buffer[i + 2] == 0xFF &&
                rx_buffer[i + 3] == 0xFF && rx_buffer[i + 4] == 0xFF && rx_buffer[i + 5] == 0xFF &&
                rx_buffer[i + 6] == 0xFF && rx_buffer[i + 7] == 0xFF)
            {
                start_index = i + 8;
                break;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Start index in create_rxba = %d", start_index);
        if (start_index < 0)
            return 0;

        int first = 40 - start_index;
        if (first >= 32)
            memcpy(rxba, &rx_buffer[start_index], 32);
        else
        {
            memcpy(rxba, &rx_buffer[start_index], first);
            memcpy(rxba + first, &rx_buffer[0], 32 - first);
        }
        return 1;
    }

    uint16_t fletcher16(uint8_t *data, size_t len)
    {
        uint16_t sum1 = 0;
        uint16_t sum2 = 0;

        for (size_t i = 0; i < len; i++)
        {
            sum1 = (sum1 + data[i]) % 255;
            sum2 = (sum2 + sum1) % 255;
        }

        return (sum2 << 8) | sum1;
    }

    void uart_rx_interrupt_loop()
    {
        while (running_ && rclcpp::ok())
        {
            memset(rx_buffer, 0, sizeof(rx_buffer));
            read_uart(rx_buffer, 40);
            if (!create_rxba())
                continue;

            uint16_t received_checksum = rxba[30] | (rxba[31] << 8);
            uint16_t calculated_checksum = fletcher16(rxba, 30);
            if (received_checksum != calculated_checksum)
            {
                // checksum failed
                // TODO: vrati ovo kad sredis checksum
                // continue;
            }

            move_status_ = rxba[0];
            double x_raw = (int32_t)((uint32_t)rxba[4] << 24 | (uint32_t)rxba[3] << 16 | (uint32_t)rxba[2] << 8 |
                                     (uint32_t)rxba[1]) /
                           10000.0;
            double y_raw = (int32_t)((uint32_t)rxba[8] << 24 | (uint32_t)rxba[7] << 16 | (uint32_t)rxba[6] << 8 |
                                     (uint32_t)rxba[5]) /
                           10000.0;
            double phi_raw = (int32_t)((uint32_t)rxba[12] << 24 | (uint32_t)rxba[11] << 16 | (uint32_t)rxba[10] << 8 |
                                       (uint32_t)rxba[9]) /
                             10000.0;
            v_base_ = (int32_t)((uint32_t)rxba[16] << 24 | (uint32_t)rxba[15] << 16 | (uint32_t)rxba[14] << 8 |
                                (uint32_t)rxba[13]) /
                      10000.0;
            w_base_ = (int32_t)((uint32_t)rxba[20] << 24 | (uint32_t)rxba[19] << 16 | (uint32_t)rxba[18] << 8 |
                                (uint32_t)rxba[17]) /
                      10000.0;

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

            // TODO: pub move_status_budz_
            auto msg = example_interfaces::msg::Int8();
            msg.data = move_status_;
            move_status_budz_pub_->publish(msg);
            // TODO: pub odom koji stigne
            publish_odometry();
        }
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
        // ovo je bas budz
        msg.twist.twist.linear.y = x_base_offs_;
        msg.twist.twist.linear.z = y_base_offs_;
        msg.twist.twist.angular.x = phi_base_offs_;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = w_base_;
        odom_pub_->publish(msg);
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

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 40;
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
        // RCLCPP_INFO(this->get_logger(), "Recieved %ld bytes.", n);
        // for (int i = 0; i < n; i++)
        //     RCLCPP_INFO(this->get_logger(), "%d", buffer[i]);
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

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

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
                // RCLCPP_INFO(this->get_logger(), "uart2 rx: %ud", byte);

                uint8_t chinch_prev = chinch_;
                chinch_ = byte & 0b1;
                auto chinch_msg = example_interfaces::msg::Bool();
                if (chinch_ && !chinch_prev)
                    chinch_msg.data = 0b1;
                else
                    chinch_msg.data = 0b0;
                chinch_pub_->publish(chinch_msg);

                switches_ = ((byte >> 1) & 0b111) | (byte & (0b1 << 4));
                auto switches_msg = example_interfaces::msg::UInt8();
                switches_msg.data = switches_;
                switches_pub_->publish(switches_msg);

                uint8_t uart2_tx =
                    vacuum_.load(std::memory_order_relaxed) | chinch_waiting_.load(std::memory_order_relaxed);
                send_uart2_byte(uart2_tx);
                // RCLCPP_INFO(this->get_logger(), "uart2 tx: %ud", uart2_tx);
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
        vacuum_.store((msg->data & 0b1111) << 1, std::memory_order_relaxed);
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

    uint8_t rxba[32];
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
    rclcpp::Subscription<ros381_interfaces::msg::MiniMBP>::SharedPtr mini_mbp_sub_;
    rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr move_status_budz_pub_;

    int8_t move_status_ = 0;
    uint8_t rx_buffer[40];
    int8_t mbp_type;
    double mbp_x;
    double mbp_y;
    double mbp_phi;
    int8_t mbp_direction;
    uint8_t mbp_obstacle;
    uint8_t mbp_v_max_100;
    uint8_t mbp_w_max_10;
    uint8_t mbp_tol_perc;
    uint8_t mbp_coeff;
    uint16_t mbp_checksum;

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
