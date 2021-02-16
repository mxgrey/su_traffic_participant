#include <functional>
#include <memory>
#include <string>
#include <map>
#include <iterator> 
#include <limits>
#include <iostream> 
#include <array> 
#include <vector> 

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>

#include <Eigen/Geometry>
#include "geometry_msgs/msg/point.hpp"
#include "su_msgs/msg/objects_location.hpp"
#include "su_msgs/msg/object.hpp"
#include "su_msgs/msg/coordinates.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>


class SuTrafficNode : public rclcpp::Node
{
public:
    static std::shared_ptr<SuTrafficNode> make();

protected:
    static int getCount() { return count++; };

private:

    void topic_callback(const su_msgs::msg::ObjectsLocation::SharedPtr msg);

    rclcpp::Subscription<su_msgs::msg::ObjectsLocation>::SharedPtr subscription_;

    static int count;

    SuTrafficNode(const rclcpp::NodeOptions& options);

    rmf_traffic_ros2::schedule::WriterPtr writer;

    rmf_utils::optional<rmf_traffic_ros2::schedule::MirrorManager> _mirror;
    rmf_utils::optional<rmf_traffic_ros2::schedule::Negotiation> _negotiation;
    
    std::map<int, Eigen::Vector3d> location_map;

    struct ParticipantInfo
    {
        std::shared_ptr<rmf_traffic::schedule::Participant> participant;
        std::shared_ptr<void> negotiation_license;
        ParticipantInfo(rmf_traffic::schedule::Participant p, rmf_traffic_ros2::schedule::Negotiation& negotiation);
    };
    std::map<std::size_t, ParticipantInfo> participant_info_map;

    void create_participant(int id, Eigen::Vector3d detectionLocation);
    void remove_participant(int id);
    void print_location_map();
    std::pair<bool, int> calculate_distance(Eigen::Vector3d newPos);

};
