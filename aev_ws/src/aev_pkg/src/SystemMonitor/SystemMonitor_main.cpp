#include "ros/ros.h"
#include "std_msgs/String.h"

#include "SystemMonitor.h"

#include "aev_pkg/driving_mode_msg.h"
#include "aev_pkg/ecu_feedback_msg.h"
#include "aev_pkg/gui_msg.h"
#include "aev_pkg/lane_detection_msg.h"
#include "aev_pkg/mpc_msg.h"
#include "aev_pkg/object_detection_msg.h"
#include "aev_pkg/radar_msg.h"
#include "aev_pkg/system_monitor_msg.h"

ros::Publisher SystemMonitor_OutputPublisher;
SystemMonitor systemMonitor;

void GuiDataCallback(const aev_pkg::gui_msg::ConstPtr& msg)
{
  systemMonitor.SystemMonitor_GUI_MsgCheck(msg->msg_counter, msg->speedSetpoint);
  //std_msgs::String string1;
  //std::stringstream ss;
  //ss << "GUI Data received: \n " << msg->msg_counter << " " << msg->speedSetpoint;
  //string1.data = ss.str();
  //ROS_INFO("%s", string1.data.c_str());
  ROS_INFO("Gui Data received.");
  if (msg->clearError)
  {
    systemMonitor.SystemMonitor_RefreshError();
    ROS_INFO("Clear error Data received.");
  }
}

void RadarDataCallback(const aev_pkg::radar_msg::ConstPtr& msg)
{
  systemMonitor.SystemMonitor_Radar_MsgCheck(msg->msg_counter);
  ROS_INFO("Radar Data received.");
}

void ObjDetcDataCallback(const aev_pkg::object_detection_msg::ConstPtr& msg)
{
  systemMonitor.SystemMonitor_ObjDetc_MsgCheck(msg->msg_counter, msg->isObject);
  if (msg->isObject)
  {
    ROS_INFO("Object in front of the car");
  }
}

void LaneDataCallback(const aev_pkg::lane_detection_msg::ConstPtr& msg)
{
  systemMonitor.SystemMonitor_LaneDetc_MsgCheck(msg->msg_counter, msg->centerOffset, msg->curvature);
}

void EcuFeedbackDataCallback(const aev_pkg::ecu_feedback_msg::ConstPtr& msg)
{
  Can_float feedbackSpeed_can;
  feedbackSpeed_can.byte.byte1 = msg->feedbackSpeed_b1;
  feedbackSpeed_can.byte.byte2 = msg->feedbackSpeed_b2;
  feedbackSpeed_can.byte.byte3 = msg->feedbackSpeed_b3;
  feedbackSpeed_can.byte.byte4 = msg->feedbackSpeed_b4;

  systemMonitor.SystemMonitor_ECU_MsgCheck(msg->msg_counter, feedbackSpeed_can.Can_Float_Value);
}

void DrivingModeDataCallback(const aev_pkg::driving_mode_msg::ConstPtr& msg)
{
  systemMonitor.SystemMonitor_DrivingMode_MsgCheck(msg->msg_counter);
}

void MPCDataCallback(const aev_pkg::mpc_msg::ConstPtr& msg)
{
  systemMonitor.SystemMonitor_MPC_MsgCheck(msg->msg_counter);
}
/*----------------------------------------------------------------------------------*/
aev_pkg::system_monitor_msg SystemMonitor_Output_msg;
void timer_main_Callback(const ros::TimerEvent& e)
{
  systemMonitor.SystemMonitor_SystemCheck();
  SystemMonitor_Output_msg.errorFlag = systemMonitor.Output.Error_flag;
  SystemMonitor_Output_msg.stopRequestFlag = systemMonitor.Output.StopRequest_flag;
  SystemMonitor_Output_msg.errorInfo = systemMonitor.Output.ErrorInfo;
  SystemMonitor_OutputPublisher.publish(SystemMonitor_Output_msg);
  ROS_INFO(".");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SystemMonitor");
  ros::NodeHandle SystemMonitor_Node;
  
  ros::Subscriber sub1 = SystemMonitor_Node.subscribe("GUI_Data", 1000, GuiDataCallback);
  ros::Subscriber sub2 = SystemMonitor_Node.subscribe("Radar_Data", 1000, RadarDataCallback);
  ros::Subscriber sub3 = SystemMonitor_Node.subscribe("ObjDetc_Data", 1000, ObjDetcDataCallback);
  ros::Subscriber sub4 = SystemMonitor_Node.subscribe("LaneDetc_Data", 1000, LaneDataCallback);
  ros::Subscriber sub6 = SystemMonitor_Node.subscribe("ECU_Feedback_Data", 1000, EcuFeedbackDataCallback);
  ros::Subscriber sub7 = SystemMonitor_Node.subscribe("DrivingMode_Data", 1000, DrivingModeDataCallback);
  ros::Subscriber sub8 = SystemMonitor_Node.subscribe("MPC_Data", 1000, MPCDataCallback);

  SystemMonitor_OutputPublisher = SystemMonitor_Node.advertise<aev_pkg::system_monitor_msg>("SystemMonitor_Data", 1000);

  ros::Timer timer_main = SystemMonitor_Node.createTimer(ros::Duration(SYS_MONITOR_MSG_TIME), timer_main_Callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
