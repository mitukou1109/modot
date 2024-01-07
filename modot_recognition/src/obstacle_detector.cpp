#include "modot_recognition/obstacle_detector.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

using namespace std::placeholders;

namespace modot_recognition
{
ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions& node_options)
  : ObstacleDetector("obstacle_detector", "", node_options)
{
}

ObstacleDetector::ObstacleDetector(const std::string& node_name, const std::string& ns,
                                   const rclcpp::NodeOptions& node_options)
  : Node(node_name, ns, node_options)
  , obstacle_detected_(false)
  , global_frame_("world")
  , leaf_size_(0.1)
  , sac_distance_threshold_(0.03)
  , sac_max_iterations_(1000)
  , sac_eps_angle_(30 * M_PI / 180)
  , sac_trials_(3)
  , min_plane_inliers_(50)
  , ground_plane_tolerance_(0.01)
  , cluster_tolerance_(0.1)
  , min_cluster_size_(10)
  , max_cluster_size_(1000)
  , obstacle_range_(1.0)
{
  this->declare_parameter("global_frame", global_frame_);
  this->declare_parameter("leaf_size", leaf_size_);
  this->declare_parameter("sac_distance_threshold", sac_distance_threshold_);
  this->declare_parameter("sac_max_iterations", sac_max_iterations_);
  this->declare_parameter("sac_eps_angle", sac_eps_angle_);
  this->declare_parameter("sac_trials", sac_trials_);
  this->declare_parameter("min_plane_inliers", static_cast<int>(min_plane_inliers_));
  this->declare_parameter("cluster_tolerance", cluster_tolerance_);
  this->declare_parameter("min_cluster_size", static_cast<int>(min_cluster_size_));
  this->declare_parameter("max_cluster_size", static_cast<int>(max_cluster_size_));
  this->declare_parameter("obstacle_range", obstacle_range_);

  this->get_parameter("global_frame", global_frame_);

  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_updater_ = std::make_unique<modot_lib::ParameterUpdater>(parameter_event_handler_);
  parameter_updater_->addParameter("leaf_size", leaf_size_);
  parameter_updater_->addParameter("sac_distance_threshold", sac_distance_threshold_);
  parameter_updater_->addParameter("sac_max_iterations", sac_max_iterations_);
  parameter_updater_->addParameter("sac_eps_angle", sac_eps_angle_);
  parameter_updater_->addParameter("sac_trials", sac_trials_);
  parameter_updater_->addParameter("min_plane_inliers", min_plane_inliers_);
  parameter_updater_->addParameter("cluster_tolerance", cluster_tolerance_);
  parameter_updater_->addParameter("min_cluster_size", min_cluster_size_);
  parameter_updater_->addParameter("max_cluster_size", max_cluster_size_);
  parameter_updater_->addParameter("obstacle_range", obstacle_range_);

  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);

  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud", 10);
  obstacle_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/detected", 1);
  obstacle_centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/obstacle_centroid", 1);
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", rclcpp::SensorDataQoS(), std::bind(&ObstacleDetector::pointCloudCallback, this, _1));

  obstacle_detected_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&ObstacleDetector::obstacleDetectedPublishTimerCallback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid.filter(*cloud);

  if (!pcl_ros::transformPointCloud(global_frame_, msg->header.stamp, *cloud, msg->header.frame_id, *cloud,
                                    *tf_buffer_))
  {
    RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud");
    return;
  }
  cloud->header.frame_id = global_frame_;

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(sac_max_iterations_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance_threshold_);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(sac_eps_angle_);

  std::vector<pcl::PointIndices::Ptr> planes;
  auto coefficients = std::make_shared<pcl::ModelCoefficients>();
  auto segmented_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*cloud, *segmented_cloud);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative(true);

  int trials = 0;
  while (segmented_cloud->width >= min_plane_inliers_)
  {
    auto inliers = std::make_shared<pcl::PointIndices>();
    seg.setInputCloud(segmented_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() >= min_plane_inliers_)
    {
      planes.push_back(inliers);

      extract.setInputCloud(segmented_cloud);
      extract.setIndices(inliers);
      extract.filter(*segmented_cloud);

      trials = 0;
    }
    else
    {
      if (++trials >= sac_trials_)
      {
        break;
      }
    }
  }

  double ground_plane_z = std::numeric_limits<double>::infinity();
  if (planes.size() > 0)
  {
    const auto& lowest_plane = *std::min_element(
        planes.begin(), planes.end(), [this, cloud](const pcl::PointIndices::Ptr& a, const pcl::PointIndices::Ptr& b) {
          return getCentroid(cloud, *a).first.z() < getCentroid(cloud, *b).first.z();
        });
    ground_plane_z = getCentroid(cloud, *lowest_plane).first.z();
    const auto& ground_plane = std::partition(
        planes.begin(), planes.end(), [cloud, ground_plane_z, this](const pcl::PointIndices::Ptr& plane) {
          return getCentroid(cloud, *plane).first.z() <= ground_plane_z + ground_plane_tolerance_;
        });
    extract.setInputCloud(cloud);
    for (auto itr = planes.begin(); itr != ground_plane; ++itr)
    {
      extract.setIndices(*itr);
      extract.filter(*cloud);
    }
  }

  if (cloud->width < min_cluster_size_)
  {
    publishEmptyPointCloud(msg->header.stamp);
    return;
  }

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
  std::vector<pcl::PointIndices> clusters;
  ece.setClusterTolerance(cluster_tolerance_);
  ece.setMinClusterSize(min_cluster_size_);
  ece.setMaxClusterSize(max_cluster_size_);
  ece.setSearchMethod(tree);
  ece.setInputCloud(cloud);
  ece.extract(clusters);

  clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                [this, cloud, ground_plane_z](const pcl::PointIndices& cluster) {
                                  return getCentroid(cloud, cluster).first.z() < ground_plane_z;
                                }),
                 clusters.end());
  if (clusters.empty())
  {
    publishEmptyPointCloud(msg->header.stamp);
    return;
  }

  auto closest_obstacle = *std::min_element(clusters.begin(), clusters.end(),
                                            [this, cloud](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                                              return getMin2DDistance(cloud, a) < getMin2DDistance(cloud, b);
                                            });

  obstacle_detected_ = getMin2DDistance(cloud, closest_obstacle) <= obstacle_range_;
  if (obstacle_detected_)
  {
    if (const auto& [centroid, valid] = getCentroid(cloud, closest_obstacle); valid)
    {
      geometry_msgs::msg::PointStamped centroid_msg;
      centroid_msg.header.stamp = msg->header.stamp;
      centroid_msg.header.frame_id = cloud->header.frame_id;
      centroid_msg.point.x = centroid.x();
      centroid_msg.point.y = centroid.y();
      centroid_msg.point.z = centroid.z();
      obstacle_centroid_pub_->publish(centroid_msg);
    }
  }

  auto closest_obstacle_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*cloud, closest_obstacle, *closest_obstacle_cloud);
  cloud = closest_obstacle_cloud;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  point_cloud_pub_->publish(cloud_msg);
}

void ObstacleDetector::obstacleDetectedPublishTimerCallback()
{
  std_msgs::msg::Bool obstacle_detected_msg;
  obstacle_detected_msg.data = obstacle_detected_;
  obstacle_detected_pub_->publish(obstacle_detected_msg);
}

void ObstacleDetector::publishEmptyPointCloud(const rclcpp::Time& stamp)
{
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(empty_cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = global_frame_;
  point_cloud_pub_->publish(cloud_msg);
}

std::pair<Eigen::Vector4f, bool> ObstacleDetector::getCentroid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                                               const pcl::PointIndices& indices)
{
  Eigen::Vector4f centroid;
  bool valid = pcl::compute3DCentroid(*cloud, indices, centroid) != 0;
  return { centroid, valid };
}

double ObstacleDetector::getMin2DDistance(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                          const pcl::PointIndices& indices)
{
  auto closest_index = *std::min_element(
      indices.indices.begin(), indices.indices.end(), [cloud](const std::size_t& a, const std::size_t& b) {
        return std::hypot(cloud->points[a].x, cloud->points[a].y) < std::hypot(cloud->points[b].x, cloud->points[b].y);
      });
  return std::hypot(cloud->points[closest_index].x, cloud->points[closest_index].y);
}
}  // namespace modot_recognition

RCLCPP_COMPONENTS_REGISTER_NODE(modot_recognition::ObstacleDetector)