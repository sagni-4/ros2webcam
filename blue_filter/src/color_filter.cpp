#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ColorFilterNode : public rclcpp::Node
{
public:
    ColorFilterNode() : Node("color_filter")
    {
        using std::placeholders::_1;

        // Use Best Effort QoS (sensor data profile)
        auto qos = rclcpp::SensorDataQoS();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image_raw", qos,
            std::bind(&ColorFilterNode::image_callback, this, _1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat original = cv_ptr->image;
        cv::Mat modified = original.clone();

        // Convert to HSV for color masking
        cv::Mat hsv;
        cv::cvtColor(modified, hsv, cv::COLOR_BGR2HSV);

        // Define blue color range
        cv::Scalar lower_blue(100, 100, 50);   // Adjust range as needed
        cv::Scalar upper_blue(140, 255, 255);

        // Create mask for blue
        cv::Mat mask;
        cv::inRange(hsv, lower_blue, upper_blue, mask);

        // Replace blue with green (BGR)
        modified.setTo(cv::Scalar(0, 255, 0), mask);

        // Display both original and modified images
        cv::imshow("Original Image", original);
        cv::imshow("Blue -> Green", modified);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorFilterNode>());
    rclcpp::shutdown();
    return 0;
}