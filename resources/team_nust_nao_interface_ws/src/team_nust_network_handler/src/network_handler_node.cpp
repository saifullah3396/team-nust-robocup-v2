#include <ros/ros.h>
#include <std_msgs/String.h>
#include "network_handler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "team_nust_network_handler_node");
  ros::NodeHandle nhp("~");
  ros::Rate rate(1);
  std::string robotIp;
  if(!nhp.getParam("robot_ip", robotIp))
    robotIp = std::string("127.0.0.1");
  ros::NodeHandle nh;
  NetworkHandler networkHandler(nh, robotIp);
  networkHandler.update();
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
