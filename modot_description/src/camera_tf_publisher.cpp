#include "modot_description/camera_tf_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::placeholders;

CameraTFPublisher::CameraTFPublisher()
  : Node("tf_publisher")
  , global_frame_("world")
  , camera_frame_("camera_link")
  , imu_frame_("imu_link")
  , camera_position_(0, 0, 0)
  , lpf_factor_(0.8)
{
  this->declare_parameter("global_frame", global_frame_);
  this->declare_parameter("camera_frame", camera_frame_);
  this->declare_parameter("imu_frame", imu_frame_);
  this->declare_parameter("lpf_factor", lpf_factor_);
  this->declare_parameter("camera_x", camera_position_[0]);
  this->declare_parameter("camera_y", camera_position_[1]);
  this->declare_parameter("camera_z", camera_position_[2]);

  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("imu_frame", imu_frame_);
  this->get_parameter("lpf_factor", lpf_factor_);
  this->get_parameter("camera_x", camera_position_[0]);
  this->get_parameter("camera_y", camera_position_[1]);
  this->get_parameter("camera_z", camera_position_[2]);

  accel_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("accel", rclcpp::SensorDataQoS(),
                                                                std::bind(&CameraTFPublisher::accelCallback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void CameraTFPublisher::accelCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  static tf2::Vector3 accel_prev;

  this->get_parameter("lpf_factor", lpf_factor_);

  tf2::Stamped<tf2::Transform> imu_to_camera_tf;
  try
  {
    tf2::fromMsg(tf_buffer_->lookupTransform(imu_frame_, camera_frame_, tf2::TimePointZero), imu_to_camera_tf);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    return;
  }

  tf2::Vector3 accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

  auto gravity = lpf_factor_ * accel_prev + (1.0 - lpf_factor_) * accel;
  accel_prev = accel;

  tf2::Quaternion global_to_imu_quat;
  global_to_imu_quat.setEuler(std::atan2(-gravity.y(), gravity.z()), 0, -M_PI_2);
  // global_to_imu_quat.setEuler(std::atan2(-gravity.y(), gravity.z()), 0,
  // std::atan2(gravity.y(), -gravity.x()));

  tf2::Transform global_to_camera_tf(global_to_imu_quat * imu_to_camera_tf.getRotation(), camera_position_);

  geometry_msgs::msg::TransformStamped global_to_camera_tf_msg;
  global_to_camera_tf_msg.header.stamp = msg->header.stamp;
  global_to_camera_tf_msg.header.frame_id = global_frame_;
  global_to_camera_tf_msg.child_frame_id = camera_frame_;
  global_to_camera_tf_msg.transform = tf2::toMsg(global_to_camera_tf);
  tf_broadcaster_->sendTransform(global_to_camera_tf_msg);
}