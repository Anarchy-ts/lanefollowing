#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>

using namespace cv;
using namespace std;
using Image = sensor_msgs::msg::Image;

class LaneFollowing : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Subscription<Image>::SharedPtr img_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr init_left_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr init_right_sub_;
    Mat frame, gray, dst;
    // Point prevpt1 = Point(239, 938);
    // Point prevpt2 = Point(1674, 943);
    Point prevpt1,prevpt2;
    Point cpt[2];
    Point fpt;
    int minlb[2];
    double ptdistance[2];
    double threshdistance[2];
    vector<double> mindistance1;
    vector<double> mindistance2;
    int error;
    bool init_flag1,init_flag2;

    void init_left_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!init_flag1) {
            prevpt1 = Point(msg->x, msg->y);
            init_flag1 = true;
        }
    }

    void init_right_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!init_flag2) {
            prevpt2 = Point(msg->x, msg->y);
            init_flag2 = true;
        }
    }

    void subs_callback(const Image::SharedPtr msg) {
        // frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        gray = cv_bridge::toCvShare(msg, "mono8")->image;

        // inRange(gray, cv::Scalar(125), cv::Scalar(135), gray);

        dst = gray(Rect(0, gray.rows / 3 * 2, gray.cols, gray.rows / 3));
        // dst = gray;
        // dst = cv_bridge::toCvShare(msg, "mono8")->image;

        Mat labels, stats, centroids;
        int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);
        if (cnt > 1) {
            for (int i = 1; i < cnt; i++) {
                double* p = centroids.ptr<double>(i);
                ptdistance[0] = abs(p[0] - prevpt1.x);
                ptdistance[1] = abs(p[0] - prevpt2.x);
                mindistance1.push_back(ptdistance[0]);
                mindistance2.push_back(ptdistance[1]);
            }

            threshdistance[0] = *min_element(mindistance1.begin(), mindistance1.end());
            threshdistance[1] = *min_element(mindistance2.begin(), mindistance2.end());

            minlb[0] = min_element(mindistance1.begin(), mindistance1.end()) - mindistance1.begin();
            minlb[1] = min_element(mindistance2.begin(), mindistance2.end()) - mindistance2.begin();

            cpt[0] = Point2d(centroids.at<double>(minlb[0] + 1, 0), centroids.at<double>(minlb[0] + 1, 1));
            cpt[1] = Point2d(centroids.at<double>(minlb[1] + 1, 0), centroids.at<double>(minlb[1] + 1, 1));

            if (threshdistance[0] > 100) cpt[0] = prevpt1;
            if (threshdistance[1] > 100) cpt[1] = prevpt2;

            mindistance1.clear();
            mindistance2.clear();
        }
        else {
            cpt[0] = prevpt1;
            cpt[1] = prevpt2;
        }

        prevpt1 = cpt[0];
        prevpt2 = cpt[1];

        fpt.x = (cpt[0].x + cpt[1].x) / 2;
        fpt.y = (cpt[0].y + cpt[1].y) / 2 + gray.rows / 3 * 2;
        cvtColor(dst, dst, COLOR_GRAY2BGR);

        circle(dst, fpt, 2, Scalar(0, 0, 255), 2);
        circle(dst, cpt[0], 2, Scalar(0, 0, 255), 2);
        circle(dst, cpt[1], 2, Scalar(255, 0, 0), 2);

        error = dst.cols / 2 - fpt.x;

        // imshow("camera", frame);
        imshow("gray", dst);
        waitKey(1);
    }

    void update_callback() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = (error*90.0/400)/15;

        cmd_vel_pub_->publish(cmd_vel);
    }

public:
    LaneFollowing()
        : Node("final"), init_flag1(false), init_flag2(false) {
        
        init_left_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/init_left", 1,
            std::bind(&LaneFollowing::init_left_callback, this, std::placeholders::_1));

        init_right_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/init_right", 1,
            std::bind(&LaneFollowing::init_right_callback, this, std::placeholders::_1));

        img_sub_ = this->create_subscription<Image>(
            "/igvc/lanes_binary", 10,
            std::bind(&LaneFollowing::subs_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/lane/cmd_vel", 10);
        update_timer_ = this->create_wall_timer(10ms, std::bind(&LaneFollowing::update_callback, this));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowing>());
    rclcpp::shutdown();
    return 0;
}
