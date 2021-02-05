#include "DetectionNode.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/executors.hpp>

std::shared_ptr<DetectionNode> DetectionNode::make(
    std::string nodeName,
    Eigen::Vector3d detectionLocation
)
{

    rclcpp::NodeOptions node_options;
    const rclcpp::Parameter use_sim_time("use_sim_time", true);
    node_options.parameter_overrides().push_back(use_sim_time);

    auto node = std::shared_ptr<DetectionNode>(new DetectionNode(nodeName, node_options));
    while(!node->_writer->ready())
        rclcpp::spin_some(node);
        
    rmf_traffic::schedule::ParticipantDescription description{
        "participant_" + nodeName,
        "scene_understanding",
        rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
        rmf_traffic::Profile{
            rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(0.1),
            rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(1.0)
        }
    };

    rmf_traffic::Trajectory t;
    using namespace std::chrono_literals;
    rmf_traffic::Duration duration_ = 600000s;
    // rmf_traffic::Time _start_time = rmf_traffic_ros2::convert(node->get_clock()->now());
    rmf_traffic::Time _start_time = rmf_traffic_ros2::convert(node->now());
    rmf_traffic::Time _finish_time = _start_time + duration_;
    //TODO: map from param
    std::string map_name = "L1";
    t.insert(_start_time, detectionLocation, {0, 0, 0});
    t.insert(_finish_time, detectionLocation, {0, 0, 0});

    node->_writer->async_make_participant(
        std::move(description),
        [node, t = std::move(t), map_name](rmf_traffic::schedule::Participant participant)
    {
        node->participant = std::move(participant);
        std::cout << "*** participant created! sending trajectory" << std::endl;
        node->participant->set({{map_name, std::move(t)}});

        rmf_traffic::schedule::StubbornNegotiator negotiator{ participant };

        auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
            *node, rmf_traffic::schedule::query_all());

        node->_mirror_manager = mirror_future.get();
        node->_negotiation = rmf_traffic_ros2::schedule::Negotiation(
            *node, node->_mirror_manager->snapshot_handle());

        // node->_negotiation->register_negotiator(participant.id(), 
        //     std::make_unique<rmf_traffic::schedule::StubbornNegotiator>(
        //     rmf_traffic::schedule::StubbornNegotiator(participant)));
        std::cout << "*** " << std::endl;
    });    


    // while (rclcpp::ok())
    // {
    //     std::cout << "*** while *** " << std::endl;
    //     rclcpp::spin_some(node);
    //     bool ready = true;
    //     ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    //     if (ready)
    //     {
    //         node->_mirror_manager = mirror_future.get();
    //         node->_negotiation = rmf_traffic_ros2::schedule::Negotiation(
    //             *node, node->_mirror_manager->snapshot_handle());
    //     }
    // }

    std::cout << "end of create *** " << std::endl;
    return node;

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Timeout while trying to connect to traffic schedule");
    // return nullptr;
}

DetectionNode::DetectionNode(std::string nodeName, const rclcpp::NodeOptions& options) 
: rclcpp::Node(nodeName, options)
{
    _writer = rmf_traffic_ros2::schedule::Writer::make(*this);
    std::cout << "*** created node: " << nodeName << std::endl;
}