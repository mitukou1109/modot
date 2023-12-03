#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  template <class T>
  void parameterCallback(const rclcpp::Parameter& p, T& out);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callback_handle_;

  double leaf_size_;
  double sac_plane_threshold_;
};
