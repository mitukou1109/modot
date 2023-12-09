#pragma once

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "modot_lib/parameter_updater.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();

private:
  static const std::vector<std::array<uint8_t, 3>> PALETTE;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  static double getMeanZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                         const pcl::PointIndices::ConstPtr& indices);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::unique_ptr<modot_lib::ParameterUpdater> parameter_updater_;

  double leaf_size_;
  double sac_threshold_;
  double plane_segmentation_ratio_;
  double cluster_tolerance_;
  uint32_t min_cluster_size_;
  uint32_t max_cluster_size_;
};
