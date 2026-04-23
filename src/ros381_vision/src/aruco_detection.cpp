#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ros381_interfaces/msg/crate.hpp"
#include "ros381_interfaces/msg/crate_stack.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <Eigen/Dense>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

class ArUcoDetection : public rclcpp::Node
{
  public:
    ArUcoDetection() : Node("aruco_detection")
    {
        this->load_parameters();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw" + topic_suffix_, 10, std::bind(&ArUcoDetection::topic_callback, this, std::placeholders::_1));

        crate_pub_ = this->create_publisher<ros381_interfaces::msg::CrateStack>("crate_stack" + topic_suffix_, 10);

        if (pub_cv_image_)
        {
            cv_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("cv_image" + topic_suffix_, 10);
        }
        else
            RCLCPP_INFO(this->get_logger(), "CV image publishing is off.");

        RCLCPP_INFO(this->get_logger(), "ArUco detection node is running. Topic suffix: [%s]", topic_suffix_);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds);

        cv::Mat bgr_img;
        cv::cvtColor(img, bgr_img, cv::COLOR_GRAY2BGR);

        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(bgr_img, markerCorners, markerIds);

            float markerLength = 0.03f;
            cv::Mat objPoints(4, 1, CV_32FC3);
            objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

            auto crate_msg = ros381_interfaces::msg::CrateStack();
            std::vector<ros381_interfaces::msg::Crate> crate_vector;

            for (size_t i = 0; i < markerIds.size(); i++)
            {
                cv::Vec3d rvec, tvec;
                cv::solvePnP(objPoints, markerCorners[i], camera_matrix_, dist_coeffs_, rvec, tvec);

                cv::Mat R_camera_marker;
                cv::Rodrigues(rvec, R_camera_marker);
                cv::Mat T_camera_marker = cv::Mat::eye(4, 4, CV_64F);
                R_camera_marker.copyTo(T_camera_marker(cv::Rect(0, 0, 3, 3)));
                T_camera_marker.at<double>(0, 3) = tvec[0];
                T_camera_marker.at<double>(1, 3) = tvec[1];
                T_camera_marker.at<double>(2, 3) = tvec[2];

                cv::Mat T_base_marker = camera2baseTF_ * T_camera_marker;

                double x = T_base_marker.at<double>(0, 3);
                double y = T_base_marker.at<double>(1, 3);
                double z = T_base_marker.at<double>(2, 3);
                cv::Mat R = T_base_marker(cv::Rect(0, 0, 3, 3));

                Eigen::Matrix3d R_eigen;
                R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0),
                    R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
                Eigen::Vector3d euler = R_eigen.eulerAngles(0, 1, 2);

                bool height_ok = true;
                bool angles_ok = true;
                if (enable_height_check_)
                {
                    double lower_limit = target_height_ - height_tolerance_;
                    double upper_limit = target_height_ + height_tolerance_;

                    height_ok = (z >= lower_limit) && (z <= upper_limit);
                }
                if (enable_angle_check_)
                {
                    bool roll_ok = (euler[0] >= -angle_tolerance_) && (euler[0] <= angle_tolerance_);
                    bool pitch_ok = (euler[1] >= -angle_tolerance_) && (euler[1] <= angle_tolerance_);
                    angles_ok = roll_ok && pitch_ok;
                }

                if (height_ok && angles_ok)
                {
                    auto crate = ros381_interfaces::msg::Crate();
                    crate.color = markerIds[i];
                    crate.x = x;
                    crate.y = y;
                    crate.phi = euler[2];
                    // RCLCPP_INFO(this->get_logger(), "Marker %d: Pos[%.3f, %.3f, %.3f] RPY[%.3f, %.3f, %.3f]",
                    // markerIds[i],
                    //             x, y, z, euler[0], euler[1], euler[2]);
                    // cv::drawFrameAxes(bgr_img, camera_matrix_, dist_coeffs_, rvec, tvec, 0.05);
                    crate_vector.push_back(crate);
                }
            }
            std::sort(crate_vector.begin(), crate_vector.end(), [](const auto &a, const auto &b) {
                    return a.y > b.y;
            });
            crate_msg.crate_list = crate_vector;
            for (auto &crate : crate_msg.crate_list)
                crate.phi = wrap(crate.phi);
            crate_msg.valid = check_crate_stack(crate_msg);
            if (crate_msg.valid)
            {
                double x_sum = 0.0, y_sum = 0.0, phi_sum = 0.0;
                for (size_t i = 0; i < 4; i++)
                {
                    x_sum += crate_msg.crate_list[i].x;
                    y_sum += crate_msg.crate_list[i].y;
                    phi_sum += crate_msg.crate_list[i].phi;
                }
                crate_msg.x = x_sum * 0.25;
                crate_msg.y = y_sum * 0.25;
                crate_msg.phi = phi_sum * 0.25;
            }
            else
            {
                crate_msg.x = 9.9;
                crate_msg.y = 9.9;
                crate_msg.phi = 9.9;
            }

            if (!crate_msg.crate_list.empty())
                crate_pub_->publish(crate_msg);
        }

        // cv::imshow("Aruco Detection", bgr_img);
        // cv::waitKey(3);

        if (pub_cv_image_)
        {
            auto msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img).toImageMsg();
            cv_image_pub_->publish(*msg_out);
        }
    }

    bool check_crate_stack(const ros381_interfaces::msg::CrateStack &crate_stack) const
    {
        const auto &crates = crate_stack.crate_list;
        if (crates.size() != 4)
        {
            return false;
        }
        double ref_angle = wrap(crates[0].phi);
        for (size_t i = 1; i < 4; i++)
        {
            double angle = wrap(crates[i].phi);
            if (std::fabs(angle - ref_angle) > angle_tolerance_)
                return false;
        }
        for (size_t i = 0; i < 3; i++)
        {
            double dx = crates[i].x - crates[i + 1].x;
            double dy = crates[i].y - crates[i + 1].y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (std::fabs(distance - 0.05) > height_tolerance_)
                return false;
        }
        return true;
    }

    double wrap(double angle) const
    {
        double wrapped_angle;
        wrapped_angle = std::fabs(angle);
        wrapped_angle -= M_PI / 2;
        wrapped_angle = std::fabs(wrapped_angle);
        return wrapped_angle;
    }

    void load_parameters()
    {
        
        this->declare_parameter<std::string>("topic_suffix", "_default");
        this->get_parameter("topic_suffix", topic_suffix_);

        this->declare_parameter<double>("target_height", 0.03);
        this->declare_parameter<double>("height_tolerance", 0.01);
        this->declare_parameter<double>("angle_tolerance", 0.1);
        this->declare_parameter<bool>("enable_height_check", true);
        this->declare_parameter<bool>("enable_angle_check", true);
        this->declare_parameter<std::vector<double>>(
            "camera_matrix", std::vector<double>{615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0});
        this->declare_parameter<std::vector<double>>("dist_coeffs", std::vector<double>{0.25, -1.4, -0.01, 0.005, 2.5});
        this->declare_parameter<bool>("pub_cv_image", true);
        this->declare_parameter<std::vector<double>>(
            "camera2base_transform", std::vector<double>{0.0, 0.707107, 0.707107, 0.164, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                         0.707107, -0.707107, 0.225, 0.0, 0.0, 0.0, 1.0});

        std::vector<double> K_vec, D_vec, flat_matrix;
        this->get_parameter("camera_matrix", K_vec);
        this->get_parameter("dist_coeffs", D_vec);
        this->get_parameter("pub_cv_image", pub_cv_image_);
        this->get_parameter("camera2base_transform", flat_matrix);

        camera_matrix_ = cv::Mat(3, 3, CV_64F, K_vec.data()).clone();
        dist_coeffs_ = cv::Mat(1, D_vec.size(), CV_64F, D_vec.data()).clone();

        camera2baseTF_ = cv::Mat::eye(4, 4, CV_64F);
        if (flat_matrix.size() == 16)
        {
            memcpy(camera2baseTF_.data, flat_matrix.data(), 16 * sizeof(double));
        }
        target_height_ = this->get_parameter("target_height").as_double();
        height_tolerance_ = this->get_parameter("height_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        enable_height_check_ = this->get_parameter("enable_height_check").as_bool();
        enable_angle_check_ = this->get_parameter("enable_angle_check").as_bool();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cv_image_pub_;
    rclcpp::Publisher<ros381_interfaces::msg::CrateStack>::SharedPtr crate_pub_;
    cv::Mat camera2baseTF_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool pub_cv_image_;
    double target_height_;
    double height_tolerance_;
    double angle_tolerance_;
    bool enable_height_check_;
    bool enable_angle_check_;
    std::string topic_suffix_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArUcoDetection>());
    rclcpp::shutdown();
    return 0;
}