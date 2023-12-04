#pragma once

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();

private:
  static const std::vector<std::array<uint8_t, 3>> PALETTE;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  template <class T>
  void parameterCallback(const rclcpp::Parameter& p, T& out);

  static double getMeanZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                         const pcl::PointIndices::ConstPtr& indices);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callback_handle_;

  double leaf_size_;
  double sac_threshold_;
  double plane_segmentation_ratio_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};
