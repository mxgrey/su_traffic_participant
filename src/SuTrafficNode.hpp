#include <functional>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>

#include <Eigen/Geometry>
#include "geometry_msgs/msg/point.hpp"
#include "su_msgs/msg/objects_location.hpp"
#include "su_msgs/msg/object.hpp"
#include "su_msgs/msg/coordinates.hpp"


#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>


class SuTrafficNode : public rclcpp::Node
{
public:
    SuTrafficNode(const rclcpp::NodeOptions& options);

    rmf_traffic_ros2::schedule::WriterPtr writer;

    rmf_utils::optional<rmf_traffic::schedule::Participant> participant[100];

protected:
    static int getCount() { return count++; };

private:

    void topic_callback(const su_msgs::msg::ObjectsLocation::SharedPtr msg);

    void print_detection_map();

    rclcpp::Subscription<su_msgs::msg::ObjectsLocation>::SharedPtr subscription_;

    static int count;

    std::map<int, Eigen::Vector3d> detections;

};
