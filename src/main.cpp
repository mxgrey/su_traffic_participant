#include "WriterNode.hpp"

#include <rclcpp/executors.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto writer_node = WriterNode::make();

  if (!writer_node)
    return 1;

  rclcpp::spin(writer_node);
  rclcpp::shutdown();
  return 0;
}