#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class CameraTFPublisher : public rclcpp::Node
{
public:
  CameraTFPublisher();

private:
  void accelCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string base_frame_;
  std::string camera_frame_;
  std::string imu_frame_;
  tf2::Vector3 camera_position_wrt_base_;
  double lpf_factor_;
};
