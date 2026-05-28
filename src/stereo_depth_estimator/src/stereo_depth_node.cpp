#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std::placeholders;

class StereoDepthNode : public rclcpp::Node {
public:
    StereoDepthNode() : Node("stereo_depth_estimator") {
        // QoS Profile tuned for high-bandwidth sensor data
        rclcpp::QoS qos_profile(10);
        qos_profile.best_effort(); 

        // Initialize Subscribers
        left_sub_.subscribe(this, "/stereo_camera/left/image_raw", qos_profile.get_rmw_qos_profile());
        right_sub_.subscribe(this, "/stereo_camera/right/image_raw", qos_profile.get_rmw_qos_profile());

        // Initialize Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        
        // Register C++20 lambda for the synchronized callback
        sync_->registerCallback(
            std::bind(&StereoDepthNode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Publisher for the disparity/depth map
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);

        // Configure OpenCV SGBM Matcher
        int minDisparity = 0;
        int numDisparities = 16 * 5; // Must be divisible by 16
        int blockSize = 5;
        
        stereo_matcher_ = cv::StereoSGBM::create(
            minDisparity, numDisparities, blockSize,
            8 * 1 * blockSize * blockSize,  // P1
            32 * 1 * blockSize * blockSize, // P2
            1, 63, 10, 100, 32, cv::StereoSGBM::MODE_SGBM
        );

        RCLCPP_INFO(this->get_logger(), "Stereo Depth Estimator initialized. Awaiting synchronized images...");
    }

private:
    // Define the synchronization policy
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    cv::Ptr<cv::StereoSGBM> stereo_matcher_;

    void stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
        try {
            // Bridge ROS messages to OpenCV Matrices (Mono8 encoding for faster processing)
            cv::Mat left_img = cv_bridge::toCvShare(left_msg, "mono8")->image;
            cv::Mat right_img = cv_bridge::toCvShare(right_msg, "mono8")->image;

            cv::Mat disparity, disparity_normalized;
            
            // Compute disparity map
            stereo_matcher_->compute(left_img, right_img, disparity);

            // Normalize for visualization (0-255)
            cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

            // Convert back to ROS message and publish
            std_msgs::msg::Header header = left_msg->header; 
            sensor_msgs::msg::Image::SharedPtr depth_msg = 
                cv_bridge::CvImage(header, "mono8", disparity_normalized).toImageMsg();
            
            depth_pub_->publish(*depth_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoDepthNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}