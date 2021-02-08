#include <functional>
#include <memory>
#include <string>

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

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(const rclcpp::NodeOptions& node_options)
  : Node("detection_subscriber", node_options)
  {
    subscription_ = this->create_subscription<su_msgs::msg::ObjectsLocation>(
      "su_detections", rclcpp::SystemDefaultsQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));  
    writer = rmf_traffic_ros2::schedule::Writer::make(*this);
  
  }

  rmf_traffic_ros2::schedule::WriterPtr writer;

protected:
   static int getCount() { return count++; }

private:
  
  void topic_callback(const su_msgs::msg::ObjectsLocation::SharedPtr msg) const
  {
    //TODO: more than 1 object per scene detected
    // for(int it = msg->objects.begin(); it != msg->objects..end(); ++it)
    // {
    //   su_msgs::msg::Object obj = msg->objects[it];
    // }
    
    RCLCPP_INFO(this->get_logger(), 
      "OBJECT ---> %s at '%f, %f, %f'", 
      msg->objects[0].object_class.c_str(), 
      msg->objects[0].object_locations[0].center[0],
      msg->objects[0].object_locations[0].center[1],
      msg->objects[0].object_locations[0].center[2]
    );

    // TODO: determine object class
    // if(msg->objects[0].object_class == "wheelchair")
    // {
      
    // }

    Eigen::Vector3d pos = Eigen::Vector3d{
      msg->objects[0].object_locations[0].center[0], msg->objects[0].object_locations[0].center[1], msg->objects[0].object_locations[0].center[2]};
    
    std::string nodeName = "detection_" + std::to_string(getCount());
    // create_participant(nodeName, pos);

  }

  rclcpp::Subscription<su_msgs::msg::ObjectsLocation>::SharedPtr subscription_;
  static int count;

};

int MinimalSubscriber::count=1;


