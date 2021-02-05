#include <rclcpp/executors.hpp>
#include "DetectionNode.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto detection_node = DetectionNode::make("z_test_single_detection", 
    Eigen::Vector3d{16.4, -6.89, -0.01}); // near entrance of pantry

  if (!detection_node)
    return 1;

  RCLCPP_INFO(detection_node->get_logger(), "Starting Detection Node");
  rclcpp::spin(detection_node);
  RCLCPP_INFO(detection_node->get_logger(), "Closing Detection Node");
  rclcpp::shutdown();
}