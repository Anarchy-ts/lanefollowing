#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber() : Node("Avoid")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed2i/zed_node/points",  // Change this to your actual topic name
            10,
            std::bind(&PointCloudSubscriber::pointcloud_callback, this, std::placeholders::_1));

        ground_height_ = 0.0;  // Adjust this value based on your ground plane height
        closest_point_ = std::make_tuple(0.0f, 0.0f, 0.0f); // Initialize to a default point
        min_distance_ = std::numeric_limits<double>::infinity();
        last_reset_time_ = this->now();
        avoid_obstacle_ = false;

        // Publisher for avoid status
        avoid_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/lanefollowing/avoid", 10);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a pointcloud message");

        // Parse point cloud data
        size_t point_step = static_cast<size_t>(msg->point_step);  // Convert point_step to size_t
        const uint8_t* data_buffer = msg->data.data();
        size_t data_size = msg->data.size();
        size_t num_points = data_size / point_step;

        // Reset closest point and distance every 2 seconds
        auto now = this->now();
        if ((now - last_reset_time_).seconds() >= 1.0) {
            min_distance_ = std::numeric_limits<double>::infinity();
            closest_point_ = std::make_tuple(0.0f, 0.0f, 0.0f);
            last_reset_time_ = now;
        }

        for (size_t i = 0; i < num_points; ++i)
        {
            const float* point_data = reinterpret_cast<const float*>(data_buffer + i * point_step);
            float x = point_data[0];
            float y = point_data[1];
            float z = point_data[2];

            // Filter points above the ground height + 20 cm and within a certain range (10x10x10)
            if (z > ground_height_ + 0.20 && x < 10 && y < 10 && z < 10)
            {
                double distance = std::sqrt(x * x + y * y + z * z);
                if (distance < min_distance_)
                {
                    min_distance_ = distance;
                    closest_point_ = std::make_tuple(x, y, z);
                }
            }
        }

        double x, y, z;
        std::tie(x, y, z) = closest_point_;
        // Check if the closest point is within 1 unit from the robot
        if (y < 0.7) {
            avoid_obstacle_ = true;
            obstacle_point_ = std::make_tuple(x, y, z);
        }
        else {
            avoid_obstacle_ = false;
        }
        RCLCPP_INFO(this->get_logger(), "Closest point at x=%f, y=%f, z=%f, distance=%f", x, y, z, min_distance_);

        // Publish the avoid status
        std_msgs::msg::Bool avoid_msg;
        avoid_msg.data = avoid_obstacle_;
        avoid_publisher_->publish(avoid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr avoid_publisher_;
    double ground_height_;
    std::tuple<float, float, float> closest_point_;
    double min_distance_;
    rclcpp::Time last_reset_time_;
    bool avoid_obstacle_;
    std::tuple<float, float, float> obstacle_point_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}
