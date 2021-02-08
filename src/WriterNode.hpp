#include "rclcpp/rclcpp.hpp"

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

class WriterNode : public rclcpp::Node
{
public:

  static std::shared_ptr<WriterNode> make();
  rmf_traffic_ros2::schedule::WriterPtr _writer;

  rmf_traffic::schedule::Participant create_participant(
      std::string id, 
      Eigen::Vector3d detectionLocation);

private:

  WriterNode(const rclcpp::NodeOptions& options);
  static std::shared_ptr<WriterNode> node;
  rmf_utils::optional<rmf_traffic_ros2::schedule::MirrorManager> _mirror_manager;
  rmf_utils::optional<rmf_traffic_ros2::schedule::Negotiation> _negotiation;

  rmf_utils::optional<rmf_traffic::schedule::Participant> participant;
};