#include "DrivingModeSel.h"
#include "aev_pkg/gui_msg.h"
#include "aev_pkg/radar_msg.h"
#include "aev_pkg/object_detection_msg.h"
#include "aev_pkg/lane_detection_msg.h"
#include "aev_pkg/system_monitor_msg.h"
#include "aev_pkg/driving_mode_msg.h"
#include "aev_pkg/ecu_feedback_msg.h"

#define CLEAR_OBJ_TIME_CNT 20

DrivingModeSel drivingModeSel;
ros::Publisher drivingModeSel_pub;

void GuiDataCallback(const aev_pkg::gui_msg::ConstPtr& msg)
{
  drivingModeSel.Input.userReqStart = msg->userReqStart;
  drivingModeSel.Input.userReqAutoRun = msg->userReqAutoRun;
  drivingModeSel.Input.userReqStop = msg->userReqStop;
  //
}

void EcuFeedbackDataCallback(const aev_pkg::ecu_feedback_msg::ConstPtr& msg)
{
  Can_float feedbackSpeed_can;
  feedbackSpeed_can.byte.byte1 = msg->feedbackSpeed_b1;
  feedbackSpeed_can.byte.byte2 = msg->feedbackSpeed_b2;
  feedbackSpeed_can.byte.byte3 = msg->feedbackSpeed_b3;
  feedbackSpeed_can.byte.byte4 = msg->feedbackSpeed_b4;
  drivingModeSel.Input.currentSpeed = feedbackSpeed_can.Can_Float_Value;
}

void RadarDataCallback(const aev_pkg::radar_msg::ConstPtr& msg)
{
  if (msg->isObject)
  {
    //drivingModeSel.Input.relativeDistance = msg->distance;
    drivingModeSel.Input.relativeDistance = 20.0f;
  }
  else
  {
    drivingModeSel.Input.relativeDistance = 20.0f;
  }
}

bool IsCameraDetcObstruction = false;
void ObjDetcDataCallback(const aev_pkg::object_detection_msg::ConstPtr& msg)
{
  IsCameraDetcObstruction = msg->isObject;
}

void LaneDataCallback(const aev_pkg::lane_detection_msg::ConstPtr& msg)
{
  drivingModeSel.Input.centerOffset = msg->centerOffset;
}

void SystemMonitorDataCallback(const aev_pkg::system_monitor_msg::ConstPtr& msg)
{
  drivingModeSel.Input.systemError = msg->errorFlag;
  drivingModeSel.Input.systemReqStop = msg->stopRequestFlag;
}

/*--------------------------10 ms cyclic--------------------------------*/
aev_pkg::driving_mode_msg drivingMode_msg;
uint32_t isObstructionTimeCount = CLEAR_OBJ_TIME_CNT;
void timer_main_Callback(const ros::TimerEvent& e)
{
  if (IsCameraDetcObstruction == true)
  {
    drivingModeSel.Input.Obstruction = true;
    isObstructionTimeCount = CLEAR_OBJ_TIME_CNT;
  }
  else
  {
    if (isObstructionTimeCount > 0)
    {
      isObstructionTimeCount--;
    }

    if (isObstructionTimeCount == 0)
    {
      drivingModeSel.Input.Obstruction = false;
    }
  }

  drivingModeSel.DrivingModeSel_MainTask();

  drivingMode_msg.msg_counter++;
  drivingMode_msg.drivingMode = drivingModeSel.Output.drivingMode;
  drivingModeSel_pub.publish(drivingMode_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DrivingModeSel");
  ros::NodeHandle DrivingModeSel_Node;
  
  //ros::Subscriber sub = DrivingModeSel_Node.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub1 = DrivingModeSel_Node.subscribe("GUI_Data", 1000, GuiDataCallback);
  ros::Subscriber sub2 = DrivingModeSel_Node.subscribe("Radar_Data", 1000, RadarDataCallback);
  ros::Subscriber sub3 = DrivingModeSel_Node.subscribe("ObjDetc_Data", 1000, ObjDetcDataCallback);
  ros::Subscriber sub4 = DrivingModeSel_Node.subscribe("LaneDetc_Data", 1000, LaneDataCallback);
  ros::Subscriber sub5 = DrivingModeSel_Node.subscribe("SystemMonitor_Data", 1000, SystemMonitorDataCallback);
  ros::Subscriber sub6 = DrivingModeSel_Node.subscribe("ECU_Feedback_Data", 1000, EcuFeedbackDataCallback);

  drivingModeSel_pub = DrivingModeSel_Node.advertise<aev_pkg::driving_mode_msg>("DrivingMode_Data", 1000);

  ros::Timer timer_main = DrivingModeSel_Node.createTimer(ros::Duration(DRIVING_MODE_MSG_TIME), timer_main_Callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
