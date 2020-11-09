#ifndef TOPIC_LISTENER_H_
#define TOPIC_LISTENER_H_

#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

namespace netft {

class Ft_listener {

public:
  Ft_listener(ros::NodeHandle &node, const std::string &topic_name);

private:
  void data_callback(const geometry_msgs::WrenchStampedConstPtr &msg);

public:
  geometry_msgs::WrenchStamped current_msg;

private:
  ros::Subscriber sub;
};

} // namespace netft

#endif
