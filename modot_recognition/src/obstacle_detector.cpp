#include "modot_recognition/obstacle_detector.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

using namespace std::placeholders;

const std::vector<std::array<uint8_t, 3>> ObstacleDetector::PALETTE = { { { 0, 255, 0 } },   { { 0, 0, 255 } },
                                                                        { { 255, 255, 0 } }, { { 255, 0, 255 } },
                                                                        { { 0, 255, 255 } }, { { 255, 255, 255 } } };

ObstacleDetector::ObstacleDetector()
  : Node("obstacle_detector")
  , global_frame_("world")
  , leaf_size_(0.05)
  , sac_threshold_(0.03)
  , plane_segmentation_ratio_(0.3)
  , cluster_tolerance_(0.1)
  , min_cluster_size_(10)
  , max_cluster_size_(10000)
  , obstacle_range_(2.0)
{
  this->declare_parameter("global_frame", global_frame_);
  this->declare_parameter("leaf_size", leaf_size_);
  this->declare_parameter("sac_threshold", sac_threshold_);
  this->declare_parameter("plane_segmentation_ratio", plane_segmentation_ratio_);
  this->declare_parameter("cluster_tolerance", cluster_tolerance_);
  this->declare_parameter("min_cluster_size", static_cast<int>(min_cluster_size_));
  this->declare_parameter("max_cluster_size", static_cast<int>(max_cluster_size_));
  this->declare_parameter("obstacle_range", obstacle_range_);

  this->get_parameter("global_frame", global_frame_);

  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_updater_ = std::make_unique<modot_lib::ParameterUpdater>(parameter_event_handler_);
  parameter_updater_->addParameter("leaf_size", leaf_size_);
  parameter_updater_->addParameter("sac_threshold", sac_threshold_);
  parameter_updater_->addParameter("plane_segmentation_ratio", plane_segmentation_ratio_);
  parameter_updater_->addParameter("cluster_tolerance", cluster_tolerance_);
  parameter_updater_->addParameter("min_cluster_size", min_cluster_size_);
  parameter_updater_->addParameter("max_cluster_size", max_cluster_size_);
  parameter_updater_->addParameter("obstacle_range", obstacle_range_);

  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud", 10);
  obstacle_centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/obstacle_centroid", 1);
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", rclcpp::SensorDataQoS(), std::bind(&ObstacleDetector::pointCloudCallback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->width >= 3)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_grid.filter(*cloud);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(sac_threshold_);

    std::vector<pcl::PointIndices::Ptr> planes;
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto segmented_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::copyPointCloud(*cloud, *segmented_cloud);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setNegative(true);

    while (segmented_cloud->width >= plane_segmentation_ratio_ * cloud->width)
    {
      auto inliers = std::make_shared<pcl::PointIndices>();
      seg.setInputCloud(segmented_cloud);
      seg.segment(*inliers, *coefficients);
      planes.push_back(inliers);

      extract.setInputCloud(segmented_cloud);
      extract.setIndices(inliers);
      extract.filter(*segmented_cloud);
    }

    if (planes.size() > 0)
    {
      const auto& ground_plane = *std::max_element(
          planes.begin(), planes.end(), [cloud](const pcl::PointIndices::Ptr& a, const pcl::PointIndices::Ptr& b) {
            return getMeanZ(cloud, *a) < getMeanZ(cloud, *b);
          });
      extract.setInputCloud(cloud);
      extract.setIndices(ground_plane);
      extract.filter(*cloud);
    }

    if (cloud->width >= min_cluster_size_)
    {
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
      tree->setInputCloud(cloud);

      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
      std::vector<pcl::PointIndices> clusters;
      ece.setClusterTolerance(cluster_tolerance_);
      ece.setMinClusterSize(min_cluster_size_);
      ece.setMaxClusterSize(max_cluster_size_);
      ece.setSearchMethod(tree);
      ece.setInputCloud(cloud);
      ece.extract(clusters);

      if (clusters.size() > 0)
      {
        for (auto itr = clusters.cbegin(); itr != clusters.cend(); ++itr)
        {
          const auto& color = PALETTE[std::distance(clusters.cbegin(), itr) % PALETTE.size()];
          for (const auto& index : itr->indices)
          {
            cloud->points[index].r = color[0];
            cloud->points[index].g = color[1];
            cloud->points[index].b = color[2];
          }
        }

        if (pcl_ros::transformPointCloud(global_frame_, msg->header.stamp, *cloud, msg->header.frame_id, *cloud,
                                         *tf_buffer_))
        {
          cloud->header.frame_id = global_frame_;

          auto closest_obstacle = *std::min_element(clusters.begin(), clusters.end(),
                                                    [cloud](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                                                      return getMin2DDistance(cloud, a) < getMin2DDistance(cloud, b);
                                                    });

          for (const auto& index : closest_obstacle.indices)
          {
            cloud->points[index].r = 255;
            cloud->points[index].g = 0;
            cloud->points[index].b = 0;
          }

          if (getMin2DDistance(cloud, closest_obstacle) < obstacle_range_)
          {
            Eigen::Vector4f centroid;
            if (pcl::compute3DCentroid(*cloud, closest_obstacle, centroid) != 0)
            {
              geometry_msgs::msg::PointStamped centroid_msg;
              centroid_msg.header.stamp = msg->header.stamp;
              centroid_msg.header.frame_id = cloud->header.frame_id;
              centroid_msg.point.x = centroid[0];
              centroid_msg.point.y = centroid[1];
              centroid_msg.point.z = centroid[2];
              obstacle_centroid_pub_->publish(centroid_msg);
            }
          }

          auto cluster_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
          pcl::copyPointCloud(*cloud, clusters, *cluster_cloud);
          cloud = cluster_cloud;
        }
      }
      else
      {
        cloud->clear();
      }
    }
    else
    {
      cloud->clear();
    }
  }
  else
  {
    cloud->clear();
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  point_cloud_pub_->publish(cloud_msg);
}

double ObstacleDetector::getMeanZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                                  const pcl::PointIndices& indices)
{
  return std::accumulate(indices.indices.begin(), indices.indices.end(), 0.0,
                         [cloud](double sum, const std::size_t& index) { return sum + cloud->points[index].z; }) /
         indices.indices.size();
}

double ObstacleDetector::getMin2DDistance(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                                          const pcl::PointIndices& indices)
{
  auto closest_index = *std::min_element(
      indices.indices.begin(), indices.indices.end(), [cloud](const std::size_t& a, const std::size_t& b) {
        return std::hypot(cloud->points[a].x, cloud->points[a].y) < std::hypot(cloud->points[b].x, cloud->points[b].y);
      });
  return std::hypot(cloud->points[closest_index].x, cloud->points[closest_index].y);
}
