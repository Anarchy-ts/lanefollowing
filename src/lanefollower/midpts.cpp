#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cmath"

geometry_msgs::msg::Point left,right,mid;

class LaneMidPointsExtractor : public rclcpp::Node
{
public:
    LaneMidPointsExtractor() : Node("midpoint")
    {
        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/igvc/lanes_binary", 10, std::bind(&LaneMidPointsExtractor::imageCallback, this, std::placeholders::_1));
        
        l_publisher = this->create_publisher<geometry_msgs::msg::Point>("/flane/leftpts", 10);
        r_publisher = this->create_publisher<geometry_msgs::msg::Point>("/flane/rightpts", 10);
        m_publisher = this->create_publisher<geometry_msgs::msg::Point>("/flane/midpts", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        const int height = msg->height;
        const int width = msg->width;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat cv_image = cv_ptr->image;
        int intensity;
        bool flagl = false, flagr = false;
        for (int y = height-150; y >= 0; y--) 
        {
            for (int x = width/2; x > 20; x--) 
            {
                intensity = cv_image.at<uchar>(y, x);
                if (intensity == 255) 
                {
                    left.x = x;
                    left.y = y;
                    left.z = 0; 
                    flagl = true;
                    break;
                }
            }
            for (int x = width/2; x <= width-20; x++) 
            {
                intensity = cv_image.at<uchar>(y, x);
                if (intensity == 255) 
                {
                    right.x = x;
                    right.y = y;
                    right.z = 0; 
                    flagr = true;
                    break;
                }
            }

            if (flagl && flagr)
            {
                mid.x = round((left.x+right.x)/2);
                mid.y = round((left.y+right.y)/2);
                mid.z = 0;
                RCLCPP_INFO_ONCE(this->get_logger(), "LEFT,RIGHT,MID publishing");
                l_publisher->publish(left);
                r_publisher->publish(right);
                m_publisher->publish(mid);
                break;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr r_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneMidPointsExtractor>());
    rclcpp::shutdown();
    return 0;
}
