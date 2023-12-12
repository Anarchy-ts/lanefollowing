#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageConverter : public rclcpp::Node {
public:
    ImageConverter() : Node("image_converter") {
        RCLCPP_INFO(get_logger(), "Publishing started");

        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/zed2i/zed_node/image_raw",
            10,
            std::bind(&ImageConverter::image_callback, this, std::placeholders::_1)
        );

        publisher_ = create_publisher<sensor_msgs::msg::Image>("/gray_image", 10);
        binary_publisher_ = create_publisher<sensor_msgs::msg::Image>("/binary_image", 10);

        cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert the ROS Image message to an OpenCV Mat
            cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Convert the image to grayscale
            cv::cvtColor(cv_image_ptr->image, gray_image_, cv::COLOR_BGR2GRAY);

            // Thresholding: Binarize the image
            cv::inRange(gray_image_, cv::Scalar(120), cv::Scalar(140), binary_image_);

            // Find contours in the binary image
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary_image_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Print the coordinates of white pixels in the binary image
            for (int row = 0; row < binary_image_.rows; ++row) {
                for (int col = 0; col < binary_image_.cols; ++col) {
                    if (binary_image_.at<uchar>(row, col) == 255) {
                        RCLCPP_INFO(get_logger(), "White pixel at: (%d, %d)", col, row);
                    }
                }
            }

      
        

            // Convert the binary image and midpoint image back to a ROS Image message
            cv_bridge_->encoding = sensor_msgs::image_encodings::MONO8;

            cv_bridge_->image = gray_image_;
            sensor_msgs::msg::Image::SharedPtr gray_msg = cv_bridge_->toImageMsg();

            cv_bridge_->image = binary_image_;
            sensor_msgs::msg::Image::SharedPtr binary_msg = cv_bridge_->toImageMsg();

           

            // Publish the grayscale, binary, and midpoint images
            publisher_->publish(*gray_msg);
            binary_publisher_->publish(*binary_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Error processing image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_publisher_;
    cv::Mat gray_image_;
    cv::Mat binary_image_;
    cv_bridge::CvImagePtr cv_bridge_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}
