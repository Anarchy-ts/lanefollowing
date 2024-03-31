#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageConverter : public rclcpp::Node
{
public:
    ImageConverter() : Node("detector")
    {
        // Subscribe to the RGB image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed2i/zed_node/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                processImage(msg);
            });

        // Publish the grayscale image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/gray_image_topic", 10);
    }

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

            // Convert RGB image to grayscale
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);
            cv::inRange(gray_image, cv::Scalar(120), cv::Scalar(140), gray_image);
            // gray_image = gray_image(cv::Rect(0, gray_image.rows / 3 * 2, gray_image.cols, gray_image.rows/3));
            // Convert OpenCV grayscale image to ROS image message
            sensor_msgs::msg::Image::SharedPtr gray_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_image).toImageMsg();

            // Publish the grayscale image
            publisher_->publish(*gray_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}
