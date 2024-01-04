#pragma once

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "modot_lib/parameter_updater.hpp"

namespace modot_recognition
{
class ObstacleDetector : public rclcpp::Node
{
public:
  explicit ObstacleDetector(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  ObstacleDetector(const std::string& node_name, const std::string& ns,
                   const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

  void publishEmptyPointCloud(const rclcpp::Time& stamp);

  std::pair<Eigen::Vector4f, bool> getCentroid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                               const pcl::PointIndices& indices);

  double getMin2DDistance(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const pcl::PointIndices& indices);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_centroid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::unique_ptr<modot_lib::ParameterUpdater> parameter_updater_;

  std::string global_frame_;
  double leaf_size_;
  double sac_distance_threshold_;
  int sac_max_iterations_;
  double sac_eps_angle_;
  int sac_trials_;
  uint32_t min_plane_inliers_;
  double ground_plane_tolerance_;
  double cluster_tolerance_;
  uint32_t min_cluster_size_;
  uint32_t max_cluster_size_;
  double obstacle_range_;
};
}  // namespace modot_recognition