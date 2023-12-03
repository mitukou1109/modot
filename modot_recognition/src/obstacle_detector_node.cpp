#include "modot_recognition/obstacle_detector.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}