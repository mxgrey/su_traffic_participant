#include "SuTrafficNode.hpp"

#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

std::shared_ptr<SuTrafficNode> SuTrafficNode::make()
{
    rclcpp::NodeOptions node_options;
    const rclcpp::Parameter use_sim_time("use_sim_time", true);
    node_options.parameter_overrides().push_back(use_sim_time);
    auto node = std::shared_ptr<SuTrafficNode>(new SuTrafficNode(node_options));

    auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    *node, rmf_traffic::schedule::query_all());

    node -> writer = rmf_traffic_ros2::schedule::Writer::make(*node);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        using namespace std::chrono_literals;
        bool ready = true;
        ready &= node->writer->ready();
        ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

        if (ready)
        {
            node->_mirror = mirror_future.get();
            node->_negotiation = rmf_traffic_ros2::schedule::Negotiation(
            *node, node->_mirror->snapshot_handle());

            return node;
        }
    }
}


SuTrafficNode::ParticipantInfo::ParticipantInfo(rmf_traffic::schedule::Participant p, rmf_traffic_ros2::schedule::Negotiation& negotiation)
    : participant(std::move(p))
{
    auto negotiator = std::make_unique<rmf_traffic::schedule::StubbornNegotiator>(participant);
    negotiation_license = negotiation.register_negotiator(participant.id(), std::move(negotiator));
}

void SuTrafficNode::create_participant(int id, Eigen::Vector3d detectionLocation)
{
    rmf_traffic::schedule::ParticipantDescription description{
        "participant_" + std::to_string(id),
        "scene_understanding",
        rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
        rmf_traffic::Profile{
            rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(0.1),
            rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(1.0)
        }
    };

    rmf_traffic::Trajectory t;
    using namespace std::chrono_literals;
    //TODO: get duration from subscriber
    rmf_traffic::Duration duration_ = 600s;
    rmf_traffic::Time _start_time = rmf_traffic_ros2::convert(this->now());
    rmf_traffic::Time _finish_time = _start_time + duration_;
    //TODO: get name from subscriber
    std::string map_name = "L1";

    t.insert(_start_time, detectionLocation, {0, 0, 0});
    t.insert(_finish_time, detectionLocation, {0, 0, 0});

    this->writer->async_make_participant(
        std::move(description),
        [this, id, t = std::move(t), map_name](rmf_traffic::schedule::Participant p)
        {
            const int p_id = p.id();
            participant_info_map.insert({ p_id, ParticipantInfo(std::move(p), *this->_negotiation) });

            RCLCPP_INFO(this->get_logger(), "Created participant with id: %d", p_id);
            participant_info_map.at(p_id).participant.set({{map_name, std::move(t)}});

            // print_location_map();
        });    
}

void SuTrafficNode::remove_participant(int id)
{
    RCLCPP_INFO(this->get_logger(), "Removing outdated waypoint");
    for (auto itr = participant_info_map.find(id); itr != participant_info_map.end(); itr++){
        itr->second.participant.clear();
        participant_info_map.erase(id);
        location_map.erase(id);
    }
}

std::pair<bool, int> SuTrafficNode::calculate_distance(Eigen::Vector3d newPos){
    bool isNearby= false;
    int p_id=0;
    double abs_dist;
    double threshold = 1.0;

    std::map<int, Eigen::Vector3d>::iterator itr; 

    for (itr = location_map.begin(); itr != location_map.end(); ++itr){
        abs_dist = sqrt(pow((itr->second[0]-newPos[0]), 2) + pow((itr->second[1]-newPos[1]), 2));
        std::cout.precision(std::numeric_limits<double>::max_digits10);
        std::cout << "distance between participants: " << std::fixed << abs_dist << std::endl;
        if (abs_dist < threshold) {
            isNearby = true; 
            p_id = itr->first;
            break;
        }
    }
    
    return std::make_pair(isNearby, p_id);
}

void SuTrafficNode::print_location_map()
{
    RCLCPP_INFO(this->get_logger(), "******** All Detections ********");
    std::map<int, Eigen::Vector3d>::iterator itr; 
    for (itr = location_map.begin(); itr != location_map.end(); ++itr){
        RCLCPP_INFO(this->get_logger(), 
            "id: %d, location: '%f %f %f'", itr->first, itr->second[0], itr->second[1], itr->second[2]);
    }
    RCLCPP_INFO(this->get_logger(), "********************************");
}


SuTrafficNode::SuTrafficNode(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("detection_subscriber", node_options)
  {
    subscription_ = this->create_subscription<su_msgs::msg::ObjectsLocation>(
      "su_detections", rclcpp::SystemDefaultsQoS(), std::bind(&SuTrafficNode::topic_callback, this, _1));  
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
            if (!location_map.empty()){
                std::pair<bool, int> isNearby = calculate_distance(pos);
                if (isNearby.first){
                    remove_participant(isNearby.second);
                    location_map[detection_id] = pos;
                    create_participant(detection_id, pos);

                } else {
                    location_map[detection_id] = pos;
                    create_participant(detection_id, pos);
                }
            } else {
                location_map[detection_id] = pos;
                create_participant(detection_id, pos);
            }
        }


    }
}

int SuTrafficNode::count=1;


