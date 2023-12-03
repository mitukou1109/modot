#include "modot_recognition/obstacle_detector.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::placeholders;

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector"), leaf_size_(0.05), sac_plane_threshold_(0.01)
{
  this->declare_parameter<double>("leaf_size", leaf_size_);
  this->declare_parameter<double>("sac_plane_threshold", sac_plane_threshold_);

  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud", 10);
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10, std::bind(&ObstacleDetector::pointCloudCallback, this, _1));

  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_callback_handle_.push_back(parameter_event_handler_->add_parameter_callback(
      "leaf_size", [this](const rclcpp::Parameter& p) { parameterCallback(p, leaf_size_); }));
  parameter_callback_handle_.push_back(parameter_event_handler_->add_parameter_callback(
      "sac_plane_threshold", [this](const rclcpp::Parameter& p) { parameterCallback(p, sac_plane_threshold_); }));
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->width == 0)
  {
    return;
  }

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid.filter(*cloud);

  auto inliers = std::make_shared<pcl::PointIndices>();
  auto coefficients = std::make_shared<pcl::ModelCoefficients>();
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_plane_threshold_);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  for (auto& index : inliers->indices)
  {
    cloud->points[index].r = 255;
    cloud->points[index].g = 0;
    cloud->points[index].b = 0;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  point_cloud_pub_->publish(cloud_msg);
}

template <class T>
void ObstacleDetector::parameterCallback(const rclcpp::Parameter& p, T& out)
{
  try
  {
    out = p.get_value<T>();
  }
  catch (const rclcpp::ParameterTypeException& e)
  {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
}