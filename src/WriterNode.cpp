#include "WriterNode.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/executors.hpp>

std::shared_ptr<WriterNode> WriterNode::make()
{
    rclcpp::NodeOptions node_options;
    const rclcpp::Parameter use_sim_time("use_sim_time", true);
    node_options.parameter_overrides().push_back(use_sim_time);

    node = std::shared_ptr<WriterNode>(new WriterNode(node_options));
    
    while(!node->_writer->ready())
        rclcpp::spin_some(node);

    std::cout << "end of create *** " << std::endl;
    return node;
}

WriterNode::WriterNode(const rclcpp::NodeOptions& options) 
: rclcpp::Node("writer_node", options)
{
    _writer = rmf_traffic_ros2::schedule::Writer::make(*this);
}

rmf_traffic::schedule::Participant WriterNode::create_participant(
      std::string nodeName, 
      Eigen::Vector3d detectionLocation)
{
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
    //TODO: get duration from subscriber
    rmf_traffic::Duration duration_ = 60s;
    rmf_traffic::Time _start_time = rmf_traffic_ros2::convert(node->now());
    rmf_traffic::Time _finish_time = _start_time + duration_;
    //TODO: get name from subscriber
    std::string map_name = "L1";

    t.insert(_start_time, detectionLocation, {0, 0, 0});
    t.insert(_finish_time, detectionLocation, {0, 0, 0});

    node->_writer->async_make_participant(
        std::move(description),
        [t = std::move(t), map_name](rmf_traffic::schedule::Participant participant)
    {
        node->participant = std::move(participant);
        std::cout << "*** participant ready ***" << std::endl;
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

        return participant;
    });    
}

std::shared_ptr<WriterNode> WriterNode::node = 
    std::shared_ptr<WriterNode>();

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto writer_node = WriterNode::make();

  if (!writer_node)
    return 1;

  rclcpp::spin(writer_node);
  std::cout << "writer node shutdown *** " << std::endl;
  rclcpp::shutdown();
  return 0;
}

