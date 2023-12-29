#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "modot_lib/parameter_updater.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();

private:
  static const std::vector<std::array<uint8_t, 3>> PALETTE;

  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obstacle_contour_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::unique_ptr<modot_lib::ParameterUpdater> parameter_updater_;

  std::string global_frame_;
  double canny_threshold_1_;
  double canny_threshold_2_;
  double min_obstacle_size_;
  double obstacle_range_;
};
