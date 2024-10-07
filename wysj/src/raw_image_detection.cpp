#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/image.hpp>  
#include <opencv2/opencv.hpp>  
#include <geometry_msgs/msg/point32.hpp>  
#include <cv_bridge/cv_bridge.h>  
  
using namespace std::chrono_literals;  
using namespace std::placeholders;  
using namespace rclcpp;  
using namespace sensor_msgs::msg;  
using namespace geometry_msgs::msg;  
  
class RawImageDetectionNode : public rclcpp::Node  
{  
public:  
    RawImageDetectionNode()  
        : Node("raw_image_detection") , click_point()  
    {  
        subscription_ = this->create_subscription<Image>("/raw_image", 10,  
            std::bind(&RawImageDetectionNode::topic_callback, this, _1));  
        publisher_ = this->create_publisher<Point32>("/click_position", 10);  
    }  
  
private:  
    void topic_callback(const Image::SharedPtr msg) const  
    {  
        cv_bridge::CvImagePtr cv_ptr;  
        try {  
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
        } catch (cv_bridge::Exception& e) {  
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());  
            return;  
        }  
  
        cv::Mat image = cv_ptr->image;  
        cv::Mat hsv;  
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);  
  
        cv::Scalar blue_low = cv::Scalar(100, 150, 0);  
        cv::Scalar blue_high = cv::Scalar(140, 255, 255);  
        cv::Mat blue_mask;  
        cv::inRange(hsv, blue_low, blue_high, blue_mask);  
  
        std::vector<std::vector<cv::Point>> blue_contours;  
        cv::findContours(blue_mask, blue_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);  
  
        cv::Rect note_rect;  
        if (!blue_contours.empty()) {  
            cv::Rect bounding_rect = cv::boundingRect(blue_contours[0]);  
            note_rect = bounding_rect;  
        }  
  
        cv::Scalar white_low = cv::Scalar(0, 0, 200);  
        cv::Scalar white_high = cv::Scalar(0, 0, 255);  
        cv::Mat white_mask;  
        cv::inRange(hsv, white_low, white_high, white_mask);  
  
        std::vector<std::vector<cv::Point>> white_contours;  
        cv::findContours(white_mask, white_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);  
  
        if (!white_contours.empty() && !note_rect.empty()) {  
            cv::RotatedRect minRect = cv::minAreaRect(cv::Mat(white_contours[0]));  
            cv::Point line_center(minRect.center.x, minRect.center.y);  
            int line_y = line_center.y;  
  
            int note_center_x = note_rect.x + note_rect.width / 2;  
            int note_center_y = note_rect.y + note_rect.height / 2;  
  
            int distance = std::abs(note_center_y - line_y);  
  
            std::string evaluation;  
            if (distance <= 15) {  
                evaluation = "perfect";  
            } else if (distance <= 30) {  
                evaluation = "good";  
            } else if (distance <= 60) {  
                evaluation = "bad";  
                return;  
            } else {  
                return;  
            }  
  
            Point32 click_point_msg;  
            click_point_msg.x = note_center_x;  
            click_point_msg.y = line_y;  
            click_point_msg.z = 0;  
  
            publisher_->publish(click_point_msg);  
        }  
    }  
  
    Subscription<Image>::SharedPtr subscription_;  
    Publisher<Point32>::SharedPtr publisher_;   
    mutable Point32 click_point;  
};  
  
int main(int argc, char * argv[]) {  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<RawImageDetectionNode>());  
    rclcpp::shutdown();  
    return 0;  
}