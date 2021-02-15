#include "SuTrafficNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    const auto node = SuTrafficNode::make();
    if (!node)
    return 1;
    RCLCPP_INFO(node->get_logger(), "Starting subscriber to SUM");
    rclcpp::spin(node);


    RCLCPP_INFO(node->get_logger(), "Closing subscriber to SUM");
    //TODO: unregister participant after trajectory duration ended
    // node->writer->unregister_participant(participant.id());
    rclcpp::shutdown();
    return 0;
}
