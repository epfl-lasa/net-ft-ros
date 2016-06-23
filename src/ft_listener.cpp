#include "netft_rdt_driver/ft_listener.h"

namespace netft
{

Ft_listener::Ft_listener(ros::NodeHandle &node, const std::string& topic_name)
{

  sub = node.subscribe(topic_name, 100,&Ft_listener::data_callback,this);
}

void Ft_listener::data_callback(const geometry_msgs::WrenchStampedConstPtr &msg)
{
  current_msg = *msg;
}

}
