#include "modot_recognition/obstacle_detector.hpp"

using namespace std::placeholders;

const std::vector<std::array<uint8_t, 3>> ObstacleDetector::PALETTE = { { { 0, 255, 0 } },   { { 0, 0, 255 } },
                                                                        { { 255, 255, 0 } }, { { 255, 0, 255 } },
                                                                        { { 0, 255, 255 } }, { { 255, 255, 255 } } };

ObstacleDetector::ObstacleDetector()
  : Node("obstacle_detector")
  , global_frame_("world")
  , canny_threshold_1_(5.0)
  , canny_threshold_2_(10.0)
  , min_obstacle_size_(5000.0)
  , obstacle_range_(2.0)
{
  this->declare_parameter("global_frame", global_frame_);
  this->declare_parameter("canny_threshold_1", canny_threshold_1_);
  this->declare_parameter("canny_threshold_2", canny_threshold_2_);
  this->declare_parameter("min_obstacle_size", min_obstacle_size_);
  this->declare_parameter("obstacle_range", obstacle_range_);

  this->get_parameter("global_frame", global_frame_);

  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_updater_ = std::make_unique<modot_lib::ParameterUpdater>(parameter_event_handler_);
  parameter_updater_->addParameter("canny_threshold_1", canny_threshold_1_);
  parameter_updater_->addParameter("canny_threshold_2", canny_threshold_2_);
  parameter_updater_->addParameter("min_obstacle_size", min_obstacle_size_);
  parameter_updater_->addParameter("obstacle_range", obstacle_range_);

  obstacle_contour_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/obstacle_contour", 1);
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth/image_raw", rclcpp::SensorDataQoS(), std::bind(&ObstacleDetector::depthImageCallback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleDetector::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
    return;
  }

  auto depth_image = cv_image->image;

  double depth_scale = 0.001;
  double min_depth = 0.3 / depth_scale;
  double max_depth = 3.0 / depth_scale;
  depth_image.setTo(min_depth, depth_image < min_depth);
  depth_image.setTo(max_depth, depth_image > max_depth);
  depth_image.convertTo(depth_image, CV_8UC1, UINT8_MAX / (max_depth - min_depth),
                        -min_depth * UINT8_MAX / (max_depth - min_depth));

  cv::Mat blurred_image;
  cv::GaussianBlur(depth_image, blurred_image, { 5, 5 }, 0);

  cv::Mat canny_image;
  cv::Canny(blurred_image, canny_image, canny_threshold_1_, canny_threshold_2_);

  cv::morphologyEx(canny_image, canny_image, cv::MORPH_DILATE,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, { 15, 15 }));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(canny_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> obstacle_contours;
  std::copy_if(
      contours.begin(), contours.end(), std::back_inserter(obstacle_contours),
      [this](const std::vector<cv::Point>& contour) { return cv::contourArea(contour) >= min_obstacle_size_; });

  cv::Mat obstacle_contour_image = cv::Mat::zeros(canny_image.size(), CV_8UC3);
  for (size_t i = 0; i < obstacle_contours.size(); i++)
  {
    auto color = PALETTE[i % PALETTE.size()];
    cv::drawContours(obstacle_contour_image, obstacle_contours, i, cv::Scalar(color[0], color[1], color[2]), 5);
  }

  cv_image->header.stamp = this->get_clock()->now();
  cv_image->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  cv_image->image = obstacle_contour_image;
  // cv_image->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  // cv_image->image = canny_image;
  obstacle_contour_pub_->publish(*cv_image->toImageMsg());
}