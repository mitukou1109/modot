#include "modot_recognition/face_detection_visualizer.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace modot_recognition
{
const std::vector<cv::Scalar> FaceDetectionVisualizer::PALETTE = { { 0, 0, 255 },    { 0, 255, 0 },   { 255, 0, 0 },
                                                                   { 255, 255, 0 },  { 255, 0, 255 }, { 0, 255, 255 },
                                                                   { 255, 255, 255 } };

FaceDetectionVisualizer::FaceDetectionVisualizer(const rclcpp::NodeOptions& node_options)
  : FaceDetectionVisualizer("face_detection_visualizer", "", node_options)
{
}

FaceDetectionVisualizer::FaceDetectionVisualizer(const std::string& node_name, const std::string& ns,
                                                 const rclcpp::NodeOptions& node_options)
  : Node(node_name, ns, node_options)
{
  result_image_pub_ =
      std::make_shared<image_transport::Publisher>(image_transport::create_publisher(this, "~/result_image"));

  image_raw_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
      this, "image_raw", std::bind(&FaceDetectionVisualizer::imageRawCallback, this, std::placeholders::_1), "raw"));

  detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "detections", 1, std::bind(&FaceDetectionVisualizer::detectionsCallback, this, std::placeholders::_1));
}

void FaceDetectionVisualizer::imageRawCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try
  {
    image_raw_ = cv_bridge::toCvShare(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void FaceDetectionVisualizer::detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
  if (!image_raw_)
  {
    return;
  }

  auto result_image = image_raw_->image.clone();

  for (std::size_t i = 0; i < msg->detections.size(); ++i)
  {
    const auto& face_bbox = msg->detections.at(i).bbox;
    const auto& face_center = face_bbox.center.position;

    auto face_upper_left = cv::Point(static_cast<int>(std::round(face_center.x - face_bbox.size_x / 2)),
                                     static_cast<int>(std::round(face_center.y - face_bbox.size_y / 2)));
    auto face_lower_left = cv::Point(static_cast<int>(std::round(face_center.x - face_bbox.size_x / 2)),
                                     static_cast<int>(std::round(face_center.y + face_bbox.size_y / 2)));
    auto face_size =
        cv::Size(static_cast<int>(std::round(face_bbox.size_x)), static_cast<int>(std::round(face_bbox.size_y)));
    auto face_name_size = cv::Size(static_cast<int>(std::round(face_bbox.size_x)), 50);

    cv::rectangle(result_image, { face_upper_left, face_size }, PALETTE.at(i % PALETTE.size()), 2);

    cv::rectangle(result_image, { face_lower_left, face_name_size }, PALETTE.at(i % PALETTE.size()), cv::FILLED);

    cv::putText(result_image, msg->detections.at(i).id, face_lower_left + cv::Point(0, 45), cv::FONT_HERSHEY_DUPLEX,
                1.5, { 0, 0, 0 }, 2);
  }

  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;
  header.frame_id = msg->header.frame_id;
  result_image_pub_->publish(cv_bridge::CvImage(header, image_raw_->encoding, result_image).toImageMsg());
}
}  // namespace modot_recognition

RCLCPP_COMPONENTS_REGISTER_NODE(modot_recognition::FaceDetectionVisualizer)