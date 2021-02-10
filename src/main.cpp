#include "SuTrafficNode.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <map>
#include <iterator> 
#include <limits>

std::shared_ptr<SuTrafficNode> node;

std::map<int, Eigen::Vector3d> map;

void print_detection_map()
{
    RCLCPP_INFO(node->get_logger(), "Detections:");
    std::map<int, Eigen::Vector3d>::iterator itr; 
    for (itr = map.begin(); itr != map.end(); ++itr){
        RCLCPP_INFO(node->get_logger(), 
            "id: %d, location: '%f %f %f'", itr->first, itr->second[0], itr->second[1], itr->second[2]);
    }
}

void create_participant(
    int id, 
    Eigen::Vector3d detectionLocation)
{
    while (!node->writer->ready())
        rclcpp::spin_some(node);

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
    rmf_traffic::Duration duration_ = 60s;
    rmf_traffic::Time _start_time = rmf_traffic_ros2::convert(node->now());
    rmf_traffic::Time _finish_time = _start_time + duration_;
    //TODO: get name from subscriber
    std::string map_name = "L1";

    t.insert(_start_time, detectionLocation, {0, 0, 0});
    t.insert(_finish_time, detectionLocation, {0, 0, 0});

    node->writer->async_make_participant(
        std::move(description),
        [id, t = std::move(t), map_name](rmf_traffic::schedule::Participant participant)
        {
            const int p_id = participant.id();
            const std::map<int, Eigen::Vector3d>::iterator it = map.find(id);
            if (it != map.end()) {
                std::swap(map[p_id], it->second);
                map.erase(it);
            }

            node->participant[p_id] = std::move(participant);
            std::cout << "*** participant ready with id: " << p_id << std::endl;
            print_detection_map();
            node->participant[p_id]->set({{map_name, std::move(t)}});
            

            

            // rmf_traffic::schedule::StubbornNegotiator negotiator{ participant };

            // auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
            // *node, rmf_traffic::schedule::query_all());

            // node->_mirror_manager = mirror_future.get();
            // node->_negotiation = rmf_traffic_ros2::schedule::Negotiation(
            // *node, node->_mirror_manager->snapshot_handle());

            // node->_negotiation->register_negotiator(participant.id(), 
            // std::make_unique<rmf_traffic::schedule::StubbornNegotiator>(
            // rmf_traffic::schedule::StubbornNegotiator(participant)));

            //TODO: unregister participant after trajectory duration ended
            // node->writer->unregister_participant(participant.id());

        });    
}

void update_participant(
    int id, 
    Eigen::Vector3d detectionLocation)
{
    while (!node->writer->ready())
        rclcpp::spin_some(node);
    std::cout << "*** clear itinerary" << std::endl;
    node->participant[id]->clear();
    map.erase(id);

}

std::pair<bool, int> calculate_distance(Eigen::Vector3d newPos){
    bool isNearby= false;
    int p_id;
    double abs_dist;
    double threshold = 1.0;

    std::map<int, Eigen::Vector3d>::iterator itr; 

    for (itr = map.begin(); itr != map.end(); ++itr){
        RCLCPP_INFO(node->get_logger(), 
            "id: %d, location: '%f %f %f'", itr->first, itr->second[0], itr->second[1], itr->second[2]);
        abs_dist = sqrt(pow((itr->second[0]-newPos[0]), 2) + pow((itr->second[1]-newPos[1]), 2));
        if (abs_dist < threshold) {
            isNearby = true; 
            p_id = itr->first;
            break;
        }
        std::cout.precision(std::numeric_limits<double>::max_digits10);
        std::cout << "*** calculated distance: " << std::fixed << abs_dist << std::endl;
    }
    
    return std::make_pair(isNearby, p_id);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    const rclcpp::Parameter use_sim_time("use_sim_time", true);
    node_options.parameter_overrides().push_back(use_sim_time);

    node = std::make_shared<SuTrafficNode>(node_options);
    RCLCPP_INFO(node->get_logger(), "Starting subscriber to SUM");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Closing subscriber to SUM");
    rclcpp::shutdown();
    return 0;
}
