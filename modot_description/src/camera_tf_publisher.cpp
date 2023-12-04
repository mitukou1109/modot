#include "modot_description/camera_tf_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

CameraTFPublisher::CameraTFPublisher()
  : Node("tf_publisher")
  , base_frame_("base_link")
  , camera_frame_("camera_link")
  , imu_frame_("imu_link")
  , camera_position_wrt_base_(0, 0, 0)
  , lpf_factor_(0.8)
{
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("camera_frame", camera_frame_);
  this->declare_parameter("imu_frame", imu_frame_);
  this->declare_parameter("lpf_factor", lpf_factor_);
  this->declare_parameter("camera_x_wrt_base", camera_position_wrt_base_[0]);
  this->declare_parameter("camera_y_wrt_base", camera_position_wrt_base_[1]);
  this->declare_parameter("camera_z_wrt_base", camera_position_wrt_base_[2]);

  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("imu_frame", imu_frame_);
  this->get_parameter("lpf_factor", lpf_factor_);
  this->get_parameter("camera_x_wrt_base", camera_position_wrt_base_[0]);
  this->get_parameter("camera_y_wrt_base", camera_position_wrt_base_[1]);
  this->get_parameter("camera_z_wrt_base", camera_position_wrt_base_[2]);

  accel_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("accel", rclcpp::QoS(10).best_effort(),
                                                                std::bind(&CameraTFPublisher::accelCallback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void CameraTFPublisher::accelCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  static tf2::Vector3 accel_prev;

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

  tf2::Vector3 accel;
  tf2::fromMsg(msg->linear_acceleration, accel);

  auto z_axis = lpf_factor_ * accel_prev + (1.0 - lpf_factor_) * accel;
  accel_prev = accel;

  tf2::Vector3 x_axis(0, 0, 1);
  auto y_axis = z_axis.cross(x_axis);

  tf2::Transform base_to_imu_tf;
  auto& rot = base_to_imu_tf.getBasis();
  rot[0] = x_axis.normalized();
  rot[1] = y_axis.normalized();
  rot[2] = z_axis.normalized();

  auto base_to_camera_tf = base_to_imu_tf * imu_to_camera_tf;
  base_to_camera_tf.setOrigin(camera_position_wrt_base_);

  geometry_msgs::msg::TransformStamped base_to_camera_tf_msg;
  base_to_camera_tf_msg.header.stamp = msg->header.stamp;
  base_to_camera_tf_msg.header.frame_id = base_frame_;
  base_to_camera_tf_msg.child_frame_id = camera_frame_;
  tf2::toMsg(base_to_camera_tf, base_to_camera_tf_msg.transform);
  tf_broadcaster_->sendTransform(base_to_camera_tf_msg);
}