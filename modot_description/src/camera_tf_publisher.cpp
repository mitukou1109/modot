#include "modot_description/camera_tf_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::placeholders;

namespace modot_description
{
CameraTFPublisher::CameraTFPublisher(const rclcpp::NodeOptions& node_options)
  : CameraTFPublisher("camera_tf_publisher", "", node_options)
{
}

CameraTFPublisher::CameraTFPublisher(const std::string& node_name, const std::string& ns,
                                     const rclcpp::NodeOptions& node_options)
  : Node(node_name, ns, node_options)
  , global_frame_("world")
  , camera_frame_("camera_link")
  , imu_frame_("imu_link")
  , lpf_factor_(0.8)
{
  this->declare_parameter("global_frame", global_frame_);
  this->declare_parameter("camera_frame", camera_frame_);
  this->declare_parameter("imu_frame", imu_frame_);
  this->declare_parameter("lpf_factor", lpf_factor_);
  this->declare_parameter("camera_x", camera_offset_.getOrigin()[0]);
  this->declare_parameter("camera_y", camera_offset_.getOrigin()[1]);
  this->declare_parameter("camera_z", camera_offset_.getOrigin()[2]);

  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("imu_frame", imu_frame_);
  this->get_parameter("lpf_factor", lpf_factor_);
  this->get_parameter("camera_x", camera_offset_.getOrigin()[0]);
  this->get_parameter("camera_y", camera_offset_.getOrigin()[1]);
  this->get_parameter("camera_z", camera_offset_.getOrigin()[2]);

  camera_vel_.stamp_ = tf2_ros::fromRclcpp(this->get_clock()->now());
  camera_vel_.frame_id_ = global_frame_;
  camera_vel_.setValue(0, 0, 0);
  camera_offset_.getBasis().setRPY(0, 0, -M_PI_2);

  parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  parameter_updater_ = std::make_unique<modot_lib::ParameterUpdater>(parameter_event_handler_);
  parameter_updater_->addParameter("lpf_factor", lpf_factor_);

  camera_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/camera_vel", 1);
  accel_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("accel", rclcpp::SensorDataQoS(),
                                                                std::bind(&CameraTFPublisher::accelCallback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void CameraTFPublisher::accelCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  static bool gravity_initialized = false;

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

  if (!gravity_initialized)
  {
    gravity_ = accel;
    gravity_initialized = true;
  }

  gravity_ = lpf_factor_ * gravity_ + (1 - lpf_factor_) * accel;

  auto rotation_axis = gravity_.cross(tf2::Vector3(0, 0, 1));
  auto global_to_camera_tf = camera_offset_ *
                             tf2::Transform(tf2::Quaternion(rotation_axis, gravity_.angle(tf2::Vector3(0, 0, 1)))) *
                             imu_to_camera_tf;

  auto global_accel = global_to_camera_tf.getBasis() * imu_to_camera_tf.inverse().getBasis() * (accel - gravity_);
  camera_vel_ += global_accel * (tf2_ros::fromMsg(msg->header.stamp) - camera_vel_.stamp_).count() / 1e9;
  camera_vel_.stamp_ = tf2_ros::fromMsg(msg->header.stamp);

  geometry_msgs::msg::TwistStamped camera_vel_msg;
  camera_vel_msg.header.stamp = msg->header.stamp;
  camera_vel_msg.header.frame_id = global_frame_;
  camera_vel_msg.twist.linear = tf2::toMsg(camera_vel_);
  camera_vel_pub_->publish(camera_vel_msg);

  geometry_msgs::msg::TransformStamped global_to_camera_tf_msg;
  global_to_camera_tf_msg.header.stamp = msg->header.stamp;
  global_to_camera_tf_msg.header.frame_id = global_frame_;
  global_to_camera_tf_msg.child_frame_id = camera_frame_;
  tf2::toMsg(global_to_camera_tf, global_to_camera_tf_msg.transform);
  tf_broadcaster_->sendTransform(global_to_camera_tf_msg);
}
}  // namespace modot_description

RCLCPP_COMPONENTS_REGISTER_NODE(modot_description::CameraTFPublisher)