#include "rclcpp/rclcpp.hpp"

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

class DetectionNode : public rclcpp::Node
{
public:

  static std::shared_ptr<DetectionNode> make(
      std::string nodeName, 
      Eigen::Vector3d detectionLocation);
  rmf_traffic_ros2::schedule::WriterPtr _writer;
  rmf_utils::optional<rmf_traffic::schedule::Participant> participant;
private:

  DetectionNode(std::string nodeName);
  // rmf_traffic_ros2::schedule::WriterPtr _writer;
  // rmf_traffic::schedule::Participant participant;
  rmf_utils::optional<rmf_traffic_ros2::schedule::MirrorManager> _mirror_manager;
  // rmf_traffic::schedule::StubbornNegotiator negotiator;
  rmf_utils::optional<rmf_traffic_ros2::schedule::Negotiation> _negotiation;
};
