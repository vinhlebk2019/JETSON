//
// File: ert_main.cpp
//
// Code generated for Simulink model 'MPC'.
//
// Model version                  : 1.23
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Mon Nov 15 23:51:36 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <math.h>

#include "../SystemDefine.h"

#include "GenCode_MPC.h" // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aev_pkg/mpc_msg.h"
#include "aev_pkg/gui_msg.h"
#include "aev_pkg/radar_msg.h"
#include "aev_pkg/object_detection_msg.h"
#include "aev_pkg/ecu_feedback_msg.h"
#include "aev_pkg/lane_detection_msg.h"
#include "aev_pkg/system_monitor_msg.h"

using namespace std::chrono;

//#define EXPORT_REPORT_DATA_CSV
//#define FPS_TEST_ENABLE

using namespace std;
#ifdef EXPORT_REPORT_DATA_CSV
#define FILE_NAME_PREFIX "Report_MPC_"
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

static MPCModelClass MPC_Obj;          // Instance of model class
MPCModelClass::ExtU_GenCode_MPC_T MPC_Input;
MPCModelClass::ExtY_GenCode_MPC_T MPC_Output;

ros::Publisher MPC_OutputPublisher;

//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(MPC_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model for base rate
  MPC_Obj.step();

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

/*----------------------------------------------------------------------------------*/

void LaneDataCallback(const aev_pkg::lane_detection_msg::ConstPtr& msg)
{
  MPC_Input.LateralDeviation = msg->centerOffset;
  MPC_Input.CurrentCurvature = msg->curvature * 0.1f;

  // Preview near the car [0]
  MPC_Input.CurvaturePreview[0] = msg->curvature * 0.1f;
  MPC_Input.CurvaturePreview[1] = msg->curvature * 0.2f;
  MPC_Input.CurvaturePreview[2] = msg->curvature * 0.2f;
  MPC_Input.CurvaturePreview[3] = msg->curvature * 0.3f;
  MPC_Input.CurvaturePreview[4] = msg->curvature * 0.4f;
  MPC_Input.CurvaturePreview[5] = msg->curvature * 0.5f;
  // Index higher number [Prediction horizon] --> Preview far from the car 

  //std::cout << "curvature " << MPC_Input.CurrentCurvature << "\r\n";
}

void ObjDetcDataCallback(const aev_pkg::object_detection_msg::ConstPtr& msg)
{
  MPC_Input.YawRate = msg->yaw_rate * 0.01f;
}

void EcuFeedbackDataCallback(const aev_pkg::ecu_feedback_msg::ConstPtr& msg)
{
  Can_float feedbackSpeed_can;
  feedbackSpeed_can.byte.byte1 = msg->feedbackSpeed_b1;
  feedbackSpeed_can.byte.byte2 = msg->feedbackSpeed_b2;
  feedbackSpeed_can.byte.byte3 = msg->feedbackSpeed_b3;
  feedbackSpeed_can.byte.byte4 = msg->feedbackSpeed_b4;
  MPC_Input.LongitudinalVelocity = feedbackSpeed_can.Can_Float_Value * cos(MPC_Output.Steeringangle);
}

/*----------------------------------------------------------------------------------*/
aev_pkg::mpc_msg mpc_output_msg;
uint32_T mpc_msg_cnt = 0;
void timer_mpc_Callback(const ros::TimerEvent& e)
{
    #ifdef FPS_TEST_ENABLE
    auto start = high_resolution_clock::now();
    auto stop = high_resolution_clock::now();
    microseconds duration = duration_cast<microseconds>(stop - start);
    #endif

    /*
    MPC_Input.LongitudinalVelocity = static_cast <float>(rand()) / static_cast <float> (RAND_MAX);
    MPC_Input.LateralDeviation = static_cast <float>(rand()) / static_cast <float> (RAND_MAX);
    MPC_Input.CurrentCurvature = 0.0f;
    MPC_Input.YawRate = static_cast <float>(rand()) / static_cast <float> (RAND_MAX);
    */
    
    MPC_Obj.setExternalInputs(&MPC_Input);

    if ((rtmGetErrorStatus(MPC_Obj.getRTM()) == (NULL)) && !rtmGetStopRequested
         (MPC_Obj.getRTM())) 
    {
      rt_OneStep();
    }

    MPC_Output = MPC_Obj.getExternalOutputs();

    mpc_msg_cnt++;
    mpc_output_msg.msg_counter = mpc_msg_cnt;
    mpc_output_msg.SteeringAngle = MPC_Output.Steeringangle;
    MPC_OutputPublisher.publish(mpc_output_msg);

    std::cout << (MPC_Input.LateralDeviation) << endl;
    std::cout << (MPC_Input.CurrentCurvature) << endl;
    std::cout << (MPC_Output.Steeringangle) << endl;
    std::cout << ("---------------------------") << endl;

    #ifdef FPS_TEST_ENABLE
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    std::cout << "Step Duration: " << duration.count() << " us \r\n";
    //std::cout << "Expected Steering Angle: " << MPC_Output.Steeringangle << " rad \r\n";
    #endif

    #ifdef EXPORT_REPORT_DATA_CSV
      std::string report_data_str;
      report_data_str = std::to_string(int(mpc_msg_cnt)) + "," 
        #ifdef FPS_TEST_ENABLE
        + std::to_string(int(duration.count())) + "," 
        #else
        + "0" + "," 
        #endif
        + std::to_string(float(MPC_Input.LateralDeviation)) + "," 
        + std::to_string(float(MPC_Output.Steeringangle));
      MyExcelFile << report_data_str << endl;
      #endif

    //ROS_INFO(".");
}

int main(int argc, char **argv)
{
  // Unused arguments
  //(void)(argc);
  //(void)(argv);

  ros::init(argc, argv, "MPC");
  ros::NodeHandle MPC_Node;
  ros::Rate loop_rate(5);

  ros::Subscriber sub1 = MPC_Node.subscribe("LaneDetc_Data", 1000, LaneDataCallback);
  ros::Subscriber sub3 = MPC_Node.subscribe("ObjDetc_Data", 1000, ObjDetcDataCallback);
  ros::Subscriber sub6 = MPC_Node.subscribe("ECU_Feedback_Data", 1000, EcuFeedbackDataCallback);

  MPC_OutputPublisher = MPC_Node.advertise<aev_pkg::mpc_msg>("MPC_Data", 1000);

  #ifdef EXPORT_REPORT_DATA_CSV
  std::string fileName = "/home/ubuntu/aev/reports/mpc/";
  fileName = fileName + FILE_NAME_PREFIX + GetCurrentTimeForFileName() + ".csv";
  MyExcelFile.open(fileName);
  MyExcelFile << "No. ,StepDuration(us), CenterOffset(m), SetSteeringangle(rad)" << endl;
  #endif

  ros::Timer timer_mpc = MPC_Node.createTimer(ros::Duration(MPC_MSG_TIME), timer_mpc_Callback);

  // Initialize model
  MPC_Obj.initialize();

  std::cout << "Started! \r\n";

  while (ros::ok())
  {
    
    loop_rate.sleep();
    ros::spinOnce();
  }

  // Disable rt_OneStep() here

  // Terminate model
  MPC_Obj.terminate();
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
