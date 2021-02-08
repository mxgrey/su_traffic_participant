#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Geometry>
#include "geometry_msgs/msg/point.hpp"
#include "su_msgs/msg/objects_location.hpp"
#include "su_msgs/msg/object.hpp"
#include "su_msgs/msg/coordinates.hpp"

#include "WriterNode.hpp"
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
    // writer->create_participant(writer, nodeName, pos);

  }


  



  rclcpp::Subscription<su_msgs::msg::ObjectsLocation>::SharedPtr subscription_;
  static int count;
};

int MinimalSubscriber::count=1;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  const rclcpp::Parameter use_sim_time("use_sim_time", true);
  node_options.parameter_overrides().push_back(use_sim_time);

  auto subscriber = std::make_shared<MinimalSubscriber>(node_options);
  RCLCPP_INFO(subscriber->get_logger(), "Starting subscriber to SUM");
  rclcpp::spin(subscriber);
  RCLCPP_INFO(subscriber->get_logger(), "Closing subscriber to SUM");
  rclcpp::shutdown();
  return 0;
}
