#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "modot_lib/parameter_updater.hpp"

namespace modot_description
{
class CameraTFPublisher : public rclcpp::Node
{
public:
  explicit CameraTFPublisher(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  CameraTFPublisher(const std::string& node_name, const std::string& ns,
                    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

private:
  void accelCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::unique_ptr<modot_lib::ParameterUpdater> parameter_updater_;

  std::string global_frame_;
  std::string camera_frame_;
  std::string imu_frame_;
  double lpf_factor_;

  tf2::Transform camera_offset_;
};
}  // namespace modot_description