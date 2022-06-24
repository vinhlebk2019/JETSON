#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aev_pkg/mpc_msg.h"
#include "aev_pkg/gui_msg.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const aev_pkg::gui_msg& msg)
{
  ROS_INFO("I heard: [%d]", msg.userReqStart);
}

void MpcDataCallback(const aev_pkg::mpc_msg::ConstPtr& msg)
{
  std::cout << "I heard Expected Steering Angle: " << msg->SteeringAngle << " rad \r\n";
  //ROS_INFO("I heard Steering angle: [%lf] rad", msg->SteeringAngle);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("GUI_Data", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("MPC_Data", 1000, MpcDataCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
