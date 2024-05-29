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
            "/igvc/lanes_binary2", 10, std::bind(&LaneMidPointsExtractor::imageCallback, this, std::placeholders::_1));
        
        m_publisher = this->create_publisher<geometry_msgs::msg::Point>("/flane/align/start", 10);

        m2_publisher = this->create_publisher<geometry_msgs::msg::Point>("/flane/align/end", 10);
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
        int start_diff = 0, end_diff = 0;
        for (int y = height-100; y >= 0; y--) 
        {
            for (int x = 20; x <= width; x++) 
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
            for (int x = width-20; x >= 0; x--) 
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
                start_diff = abs(left.x-right.x);
                if (start_diff>10)
                {
                    mid.x = round((left.x+right.x)/2);
                    mid.y = round((left.y+right.y)/2);
                    mid.z = 0;
                    RCLCPP_INFO_ONCE(this->get_logger(), "LEFT1,RIGHT1,MID1 publishing");
                    m_publisher->publish(mid);
                    break;
                }
                else
                {
                    flagl = false; 
                    flagr = false;
                    RCLCPP_INFO_ONCE(this->get_logger(), "RETRY");
                }

            }
        }
        flagl = false; 
        flagr = false;
        for (int y = 0; y <= height-100; y++) 
        {
            for (int x = 20; x <= width; x++) 
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
            for (int x = width-20; x >= 0; x--) 
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
                end_diff = abs(left.x-right.x);
                
                if (end_diff>10)
                {
                    mid.x = round((left.x+right.x)/2);
                    mid.y = round((left.y+right.y)/2);
                    mid.z = 0;
                    RCLCPP_INFO_ONCE(this->get_logger(), "LEFT2,RIGHT2,MID2 publishing");
                    m2_publisher->publish(mid);
                    break;                    
                }
                else
                {
                    flagl = false; 
                    flagr = false;
                    RCLCPP_INFO_ONCE(this->get_logger(), "RETRY");
                }

            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr r_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l2_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr r2_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m2_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneMidPointsExtractor>());
    rclcpp::shutdown();
    return 0;
}
