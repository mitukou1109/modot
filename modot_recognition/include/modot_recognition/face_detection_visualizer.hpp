#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace modot_recognition
{
class FaceDetectionVisualizer : public rclcpp::Node
{
public:
  explicit FaceDetectionVisualizer(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  FaceDetectionVisualizer(const std::string& node_name, const std::string& ns,
                          const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

private:
  static const std::vector<cv::Scalar> PALETTE;

  void imageRawCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  void detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

  cv_bridge::CvImageConstPtr image_raw_;

  std::shared_ptr<image_transport::Publisher> result_image_pub_;

  std::shared_ptr<image_transport::Subscriber> image_raw_sub_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
};
}  // namespace modot_recognition