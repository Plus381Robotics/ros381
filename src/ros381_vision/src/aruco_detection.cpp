#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

cv::Mat image_processing(const cv::Mat in_image);

class ArUcoDetection : public rclcpp::Node
{
  public:
    ArUcoDetection() : Node("aruco_detection")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ArUcoDetection::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cv_image", 10);

        RCLCPP_INFO(this->get_logger(), "ArUco detection node is running.");
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;
        cv::Mat gray_img;
        cv::cvtColor(img, gray_img, cv::COLOR_RGB2GRAY);

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        cv::aruco::detectMarkers(gray_img, dictionary, markerCorners, markerIds);

        cv::Mat out_img = img.clone();
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(out_img, markerCorners, markerIds);
            }

        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, out_img);
        sensor_msgs::msg::Image out_msg;
        img_bridge.toImageMsg(out_msg);

        publisher_->publish(out_msg);

        cv::Mat display_image;
        cv::cvtColor(out_img, display_image, cv::COLOR_RGB2BGR);
        cv::imshow("Aruco Detection", display_image);
        cv::waitKey(3);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArUcoDetection>());
    rclcpp::shutdown();
    return 0;
}