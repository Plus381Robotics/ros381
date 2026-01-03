#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include "cv_bridge/cv_bridge.h"
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

class ArUcoDetection : public rclcpp::Node
{
  public:
    ArUcoDetection() : Node("aruco_detection")
    {
        this->declare_parameter<std::vector<double>>(
            "camera_matrix", std::vector<double>{615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0});

        this->declare_parameter<std::vector<double>>("dist_coeffs", std::vector<double>{0.25, -1.4, -0.01, 0.005, 2.5});

        std::vector<double> K_vec, D_vec;
        this->get_parameter("camera_matrix", K_vec);
        this->get_parameter("dist_coeffs", D_vec);

        camera_matrix_ = cv::Mat(3, 3, CV_64F, K_vec.data()).clone();
        dist_coeffs_ = cv::Mat(1, D_vec.size(), CV_64F, D_vec.data()).clone();

        this->setTransform();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&ArUcoDetection::topic_callback, this, std::placeholders::_1));

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

                RCLCPP_INFO(this->get_logger(), "Marker %d: Pos[%.3f, %.3f, %.3f] RPY[%.3f, %.3f, %.3f]", markerIds[i],
                            x, y, z, euler[0], euler[1], euler[2]);
                cv::drawFrameAxes(bgr_img, camera_matrix_, dist_coeffs_, rvec, tvec, 0.05);
            }
        }

        cv::imshow("Aruco Detection", bgr_img);
        cv::waitKey(3);
    }

    void setTransform()
    {
        camera2baseTF_ = cv::Mat::eye(4, 4, CV_64F);
        this->declare_parameter<std::vector<double>>(
            "camera2base_transform", std::vector<double>{0.0, 0.707107, 0.707107, 0.164, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                         0.707107, -0.707107, 0.225, 0.0, 0.0, 0.0, 1.0});
        std::vector<double> flat_matrix = get_parameter("camera2base_transform").as_double_array();

        if (flat_matrix.size() == 16)
        {
            memcpy(camera2baseTF_.data, flat_matrix.data(), 16 * sizeof(double));
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::Mat camera2baseTF_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArUcoDetection>());
    rclcpp::shutdown();
    return 0;
}