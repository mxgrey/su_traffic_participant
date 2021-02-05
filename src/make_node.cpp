#include <rclcpp/executors.hpp>
#include "DetectionNode.hpp"

void make_node(std::string nodeName, Eigen::Vector3d pos)
{
    const auto detection_node = DetectionNode::make(nodeName, pos);

    rclcpp::ExecutorOptions options;
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(detection_node);
    // while (!finished && rclcpp::ok(options.context))
    executor.spin_some();
}