#include "SuTrafficNode.hpp"
#include "ParticipantInterface.hpp"

#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

SuTrafficNode::SuTrafficNode(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("detection_subscriber", node_options)
  {
    subscription_ = this->create_subscription<su_msgs::msg::ObjectsLocation>(
      "su_detections", rclcpp::SystemDefaultsQoS(), std::bind(&SuTrafficNode::topic_callback, this, _1));  
    writer = rmf_traffic_ros2::schedule::Writer::make(*this);
  
}

void SuTrafficNode::topic_callback(const su_msgs::msg::ObjectsLocation::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), 
        "OBJECT ---> %s at '%f, %f, %f'", 
        msg->objects[0].object_class.c_str(), 
        msg->objects[0].object_locations[0].center[0],
        msg->objects[0].object_locations[0].center[1],
        msg->objects[0].object_locations[0].center[2]
    );

    Eigen::Vector3d pos = Eigen::Vector3d{
        msg->objects[0].object_locations[0].center[0], msg->objects[0].object_locations[0].center[1], msg->objects[0].object_locations[0].center[2]};

    std::string nodeName = "detection_" + std::to_string(getCount());
    std::cout << "*** " << nodeName << std::endl;
    create_participant(nodeName, pos);
    // test();

};

int SuTrafficNode::count=1;


