#include "SuTrafficNode.hpp"
#include "ParticipantInterface.hpp"

#include "std_msgs/msg/string.hpp"

#include <iostream> 
#include <array> 
#include <vector> 


using std::placeholders::_1;

SuTrafficNode::SuTrafficNode(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("detection_subscriber", node_options)
  {
    subscription_ = this->create_subscription<su_msgs::msg::ObjectsLocation>(
      "su_detections", rclcpp::SystemDefaultsQoS(), std::bind(&SuTrafficNode::topic_callback, this, _1));  
    writer = rmf_traffic_ros2::schedule::Writer::make(*this);
  
}

void SuTrafficNode::topic_callback(const su_msgs::msg::ObjectsLocation::SharedPtr msg)
{
    for(su_msgs::msg::Object obj : msg->objects)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Msg received ---> %s detected at '%f, %f, %f'", 
            obj.object_class.c_str(), obj.object_locations[0].center[0], obj.object_locations[0].center[1], obj.object_locations[0].center[2]);

        bool isObjClassValid = false;
        std::array<std::string, 2> valid_obj_classes = {"wheelchair", "cone"};
        for(auto &valid_obj_class : valid_obj_classes) {   
            if (obj.object_class == valid_obj_class) {
                isObjClassValid = true;
            }
        }

        if(!isObjClassValid){
            RCLCPP_INFO(this->get_logger(), "Detection ignored");
            continue;
        } else{
            int detection_id = getCount();
            Eigen::Vector3d pos = Eigen::Vector3d{obj.object_locations[0].center[0], obj.object_locations[0].center[1], obj.object_locations[0].center[2]};
        
            //TODO: write shorter codes
            if (!map.empty()){
                std::pair<bool, int> isNearby = calculate_distance(pos);
                if (isNearby.first){
                    remove_participant(isNearby.second);
                    map[detection_id] = pos;
                    create_participant(detection_id, pos);

                } else {
                    map[detection_id] = pos;
                    create_participant(detection_id, pos);
                }
            } else {
                map[detection_id] = pos;
                create_participant(detection_id, pos);
            }
        }


    }
};

int SuTrafficNode::count=1;


