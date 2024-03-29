#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bits/stdc++.h"

bool flag1 = false, flag2 = false;
geometry_msgs::msg::Point left,right;

class LanePointsExtractor : public rclcpp::Node
{
public:
    LanePointsExtractor() : Node("initPointExtract")
    {
        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/gray_image_topic", 10, std::bind(&LanePointsExtractor::imageCallback, this, std::placeholders::_1));
        
        l_publisher = this->create_publisher<geometry_msgs::msg::Point>("/init_left", 10);
        r_publisher = this->create_publisher<geometry_msgs::msg::Point>("/init_right", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        const int height = msg->height;
        const int width = msg->width;
        const int step = msg->step;

        const uint8_t* data = msg->data.data();
        if (!flag1 || !flag2)
        {
            for (int y = height-10; y >= 0; y--) 
            {
                for (int x = 0; x < width/2; x++) 
                {
                    int index = y * step + x;

                    uint8_t intensity = msg->data[index];

                    if (intensity == 255) 
                    {
                        geometry_msgs::msg::Point pixel_point;
                        pixel_point.x = x;
                        pixel_point.y = y;
                        pixel_point.z = 0; 

                        // l_publisher->publish(pixel_point);
                        left = pixel_point;
                        flag1 = true;
                        break;
                        
                    }
                }
                for (int x = width; x >= width/2; x--) 
                {
                    int index = y * step + x;

                    uint8_t intensity = msg->data[index];

                    if (intensity == 255) 
                    {
                        geometry_msgs::msg::Point pixel_point;
                        pixel_point.x = x;
                        pixel_point.y = y;
                        pixel_point.z = 0; 

                        // r_publisher->publish(pixel_point);
                        right = pixel_point;
                        flag2 = true;
                        break;
                        
                    }
                }
                if(flag1 && flag2)
                    break;
            }
        }
        else
        {
            l_publisher->publish(left);
            r_publisher->publish(right);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr r_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LanePointsExtractor>());
    rclcpp::shutdown();
    return 0;
}