#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../SystemDefine.h"

#include "aev_pkg/driving_mode_msg.h"
#include "aev_pkg/ecu_feedback_msg.h"
#include "aev_pkg/gui_msg.h"
#include "aev_pkg/lane_detection_msg.h"
#include "aev_pkg/mpc_msg.h"
#include "aev_pkg/object_detection_msg.h"
#include "aev_pkg/radar_msg.h"
#include "aev_pkg/system_monitor_msg.h"

#define EXPORT_REPORT_DATA_CSV
#define PRINT_CONSOLE_ENABLE

using namespace std;
#ifdef EXPORT_REPORT_DATA_CSV
#define FILE_NAME_PREFIX "Report_SysMonitor_"
uint32_t data_no = 0;
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s; 
}
#endif

#ifdef EXPORT_REPORT_DATA_CSV
  ofstream MyExcelFile;
#endif

ros::Publisher test_all_pub;

aev_pkg::driving_mode_msg drivingMode_msg;
aev_pkg::ecu_feedback_msg ecuFeedback_msg;
aev_pkg::gui_msg Gui_msg;
aev_pkg::lane_detection_msg laneDetection_msg;
aev_pkg::mpc_msg Mpc_msg;
aev_pkg::object_detection_msg objectDetection_msg;
aev_pkg::radar_msg Radar_msg;
aev_pkg::system_monitor_msg systemMonitor_msg;

void GuiDataCallback(const aev_pkg::gui_msg::ConstPtr& msg)
{
  Gui_msg.msg_counter = msg->msg_counter;
  Gui_msg.userReqStart = msg->userReqStart;
  Gui_msg.userReqAutoRun = msg->userReqAutoRun;
  Gui_msg.userReqStop = msg->userReqStop;
  Gui_msg.speedSetpoint = msg->speedSetpoint;
  Gui_msg.turnSignal = msg->turnSignal;
  Gui_msg.horn = msg->horn;
  Gui_msg.frontLight = msg->frontLight;
  Gui_msg.steeringLeftRight = msg->steeringLeftRight;
}

float ECUfeedbackSpeed = 0.0;
void EcuFeedbackDataCallback(const aev_pkg::ecu_feedback_msg::ConstPtr& msg)
{
  Can_float feedbackSpeed_can;
  feedbackSpeed_can.byte.byte1 = msg->feedbackSpeed_b1;
  feedbackSpeed_can.byte.byte2 = msg->feedbackSpeed_b2;
  feedbackSpeed_can.byte.byte3 = msg->feedbackSpeed_b3;
  feedbackSpeed_can.byte.byte4 = msg->feedbackSpeed_b4;
  ECUfeedbackSpeed = feedbackSpeed_can.Can_Float_Value;

  ecuFeedback_msg.msg_counter = msg->msg_counter;
  ecuFeedback_msg.acceleratorLevel = msg->acceleratorLevel;
  ecuFeedback_msg.acceleratorSwitch = msg->acceleratorSwitch;
  ecuFeedback_msg.brakeSwitch = msg->brakeSwitch;
  ecuFeedback_msg.movingDirection = msg->movingDirection;
  ecuFeedback_msg.turnSignal = msg->turnSignal;
  ecuFeedback_msg.horn = msg->horn;
  ecuFeedback_msg.frontLight = msg->frontLight;
}

void RadarDataCallback(const aev_pkg::radar_msg::ConstPtr& msg)
{
  Radar_msg.msg_counter = msg->msg_counter;
  Radar_msg.isObject = msg->isObject;
  Radar_msg.distance = msg->distance;

  Radar_msg.ttcSpeed = msg->ttcSpeed;
  Radar_msg.ttcSteering = msg->ttcSteering;
  Radar_msg.ttcKey = msg->ttcKey;

}

void ObjDetcDataCallback(const aev_pkg::object_detection_msg::ConstPtr& msg)
{
  objectDetection_msg.msg_counter = msg->msg_counter;
  objectDetection_msg.isObject = msg->isObject;
  objectDetection_msg.yaw_rate = msg->yaw_rate;
}

void LaneDataCallback(const aev_pkg::lane_detection_msg::ConstPtr& msg)
{
  laneDetection_msg.msg_counter = msg->msg_counter;
  laneDetection_msg.centerOffset = msg->centerOffset;
  laneDetection_msg.curvature = msg->curvature;
}

void SystemMonitorDataCallback(const aev_pkg::system_monitor_msg::ConstPtr& msg)
{
  systemMonitor_msg.errorFlag = msg->errorFlag;
  systemMonitor_msg.stopRequestFlag = msg->stopRequestFlag;
  systemMonitor_msg.errorInfo = msg->errorInfo;
}

void DrivingModeDataCallback(const aev_pkg::driving_mode_msg::ConstPtr& msg)
{
  drivingMode_msg.msg_counter = msg->msg_counter;
  drivingMode_msg.drivingMode = msg->drivingMode;
}

void MPCDataCallback(const aev_pkg::mpc_msg::ConstPtr& msg)
{
  Mpc_msg.msg_counter = msg->msg_counter;
  Mpc_msg.SteeringAngle = msg->SteeringAngle;
}

void timer_main_Callback(const ros::TimerEvent& e)
{
  #ifdef PRINT_CONSOLE_ENABLE
  if (systemMonitor_msg.errorFlag || systemMonitor_msg.stopRequestFlag)
  {
    std::cout << "*** I See System Error : " << (int)systemMonitor_msg.errorInfo << "\n\n";
  }

  std::cout << "objectDetection_msg.isObject  = " << (int)objectDetection_msg.isObject << "\n";
  std::cout << "objectDetection_msg.yaw_rate  = " << (float)objectDetection_msg.yaw_rate << "\n";
  
  std::cout << "Radar_msg.isObject            = " << (int)Radar_msg.isObject << "\n";
  std::cout << "Radar_msg.distance            = " << (float)Radar_msg.distance << "\n";
  std::cout << "Radar_msg.ttcSpeed            = " << (float)Radar_msg.ttcSpeed << "\n";
  std::cout << "Radar_msg.ttcSteering         = " << (float)Radar_msg.ttcSteering << "\n";
  std::cout << "Radar_msg.ttcKey              = " << Radar_msg.ttcKey << "\n";

  std::cout << "laneDetection_msg.centerOffset= " << (float)laneDetection_msg.centerOffset << "\n";
  std::cout << "Mpc_msg.SteeringAngle         = " << (float)Mpc_msg.SteeringAngle << "\n";
  std::cout << "drivingMode_msg.drivingMode   = " << (int)drivingMode_msg.drivingMode << "\n";
  
  std::cout << "ecuFeedback_msg.accelerSwitch = " << (int)ecuFeedback_msg.acceleratorSwitch << "\n";
  std::cout << "ecuFeedback_msg.brakeSwitch   = " << (int)ecuFeedback_msg.brakeSwitch << "\n";
  std::cout << "ecuFeedback_msg.feedbackSpeed = " << (float)ECUfeedbackSpeed << "\n";
  std::cout << "Gui_msg.speedSetpoint         = " << (int)Gui_msg.speedSetpoint << "\n";
  
  std::cout << "systemMonitor_msg.errorFlag   = " << (int)systemMonitor_msg.errorFlag << "\n";
  std::cout << "systemMonitor_msg.stopReqFlag = " << (int)systemMonitor_msg.stopRequestFlag << "\n";
  std::cout << "systemMonitor_msg.errorInfo   = " << (int)systemMonitor_msg.errorInfo << "\n";
  std::cout << "---------------------------------------------------------" << "\n";
  #endif

  #ifdef EXPORT_REPORT_DATA_CSV
  std::string report_data_str;
  data_no++;
  report_data_str = std::to_string((int)data_no) + ","
                  + std::to_string((int)objectDetection_msg.isObject) + ","
                  + std::to_string((float)objectDetection_msg.yaw_rate) + ","
                  + std::to_string((int)Radar_msg.isObject) + ","
                  + std::to_string((float)Radar_msg.distance) + ","
                  + std::to_string((float)laneDetection_msg.centerOffset) + ","
                  + std::to_string((float)Mpc_msg.SteeringAngle) + ","
                  + std::to_string((int)drivingMode_msg.drivingMode) + ","
                  + std::to_string((int)ecuFeedback_msg.acceleratorSwitch) + ","
                  + std::to_string((int)ecuFeedback_msg.brakeSwitch) + ","
                  + std::to_string((float)ECUfeedbackSpeed) + ","
                  + std::to_string((int)Gui_msg.speedSetpoint) + ","
                  + std::to_string((int)systemMonitor_msg.errorFlag) + ","
                  + std::to_string((int)systemMonitor_msg.stopRequestFlag) + ","
                  + std::to_string((int)systemMonitor_msg.errorInfo);
  MyExcelFile << report_data_str << endl;
  #endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_all");
  ros::NodeHandle test_all_Node;
  
  #ifdef EXPORT_REPORT_DATA_CSV
  std::string fileName = "/home/ubuntu/aev/reports/All/";
  fileName = fileName + FILE_NAME_PREFIX + GetCurrentTimeForFileName() + ".csv";
  MyExcelFile.open(fileName);
  MyExcelFile << "No. ,\
                Camera.isObject, \
                yaw_rate, \
                Radar.isObject, \
                Radar.distance, \
                centerOffset, \
                SteeringAngle(rad), \
                drivingMode, \
                accelerSwitch, \
                brakeSwitch, \
                feedbackSpeed, \
                speedSetpoint, \
                errorFlag, \
                stopReqFlag, \
                errorInfo" << endl;
  #endif

  //ros::Subscriber sub = test_all_Node.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub1 = test_all_Node.subscribe("GUI_Data", 1000, GuiDataCallback);
  ros::Subscriber sub2 = test_all_Node.subscribe("Radar_Data", 1000, RadarDataCallback);
  ros::Subscriber sub3 = test_all_Node.subscribe("ObjDetc_Data", 1000, ObjDetcDataCallback);
  ros::Subscriber sub4 = test_all_Node.subscribe("LaneDetc_Data", 1000, LaneDataCallback);
  ros::Subscriber sub5 = test_all_Node.subscribe("SystemMonitor_Data", 1000, SystemMonitorDataCallback);
  ros::Subscriber sub6 = test_all_Node.subscribe("ECU_Feedback_Data", 1000, EcuFeedbackDataCallback);
  ros::Subscriber sub7 = test_all_Node.subscribe("DrivingMode_Data", 1000, DrivingModeDataCallback);
  ros::Subscriber sub8 = test_all_Node.subscribe("MPC_Data", 1000, MPCDataCallback);

  //drivingModeSel_pub = test_all_Node.advertise<aev_pkg::driving_mode_msg>("DrivingMode_Data", 1000);
  ros::Timer timer_main = test_all_Node.createTimer(ros::Duration(0.1), timer_main_Callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
