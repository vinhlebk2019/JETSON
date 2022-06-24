#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <ros/ros.h>
#include "../SystemDefine.h"

/* User define parameters */
#define SPEED_DEVIATION 2.0
#define SPEED_CONTROL_ERROR_TIME_CNT 300

#define LANE_OFFSET_THRES 0.3 /* meter */
#define LANE_OFFSET_MAX_THRES 1.0 /* meter */
#define CENTER_OFFSET_ERROR_TIME_CNT 20

#define CURVATURE_ERROR_THRES 2
#define CURVATURE_ERROR_TIME_CNT 20

#define OBJECT_DETC_NOISE_ERROR_CNT 5

typedef enum
{
  Radar_MsgCheck = 0,
  ECU_MsgCheck,
  GUI_MsgCheck,
  LaneDetc_MsgCheck,
  ObjDetc_MsgCheck,
  DrivingMode_MsgCheck,
  MPC_MsgCheck,
  msg_type_last,
} msg_type;

typedef struct
{
  bool Error_flag = false;
  bool StopRequest_flag = false;
  SystemMonitor_ErrorInfo ErrorInfo = NoError;
} SystemMonitor_Output;

class SystemMonitor
{
  public:
    SystemMonitor(void);
    // explicit
    SystemMonitor(SystemMonitor &&) {}
    // implicit
    SystemMonitor(const SystemMonitor&) = default;
    SystemMonitor& operator=(const SystemMonitor&) = default;
    ~SystemMonitor();

    SystemMonitor_Output Output;

    /* Cyclic function to check message of components in system */
    void SystemMonitor_Radar_MsgCheck(uint32_t msg_cnt);
    void SystemMonitor_ECU_MsgCheck(uint32_t msg_cnt, float current_speed);
    void SystemMonitor_GUI_MsgCheck(uint32_t msg_cnt, int16_t speed_setpoint);
    void SystemMonitor_LaneDetc_MsgCheck(uint32_t msg_cnt, float center_offset, float curvature);
    void SystemMonitor_ObjDetc_MsgCheck(uint32_t msg_cnt, bool isObject);
    void SystemMonitor_DrivingMode_MsgCheck(uint32_t msg_cnt);
    void SystemMonitor_MPC_MsgCheck(uint32_t msg_cnt);

    void SystemMonitor_MsgTimeCheck(void);

    /* Main function to process the error of system */
    void SystemMonitor_SystemCheck(void);
    void SystemMonitor_RefreshError(void);

  private:
    /* Variables for message timing check */
    uint32_t msg_time_xbase[msg_type_last] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};
    uint32_t msg_time_xbase_thres[msg_type_last] = {250u, 250u, 250u, 250u, 250u, 250u, 250u}; // 250*0.2s = 5s
    bool new_msg_flag[msg_type_last] = {false, false, false, false, false, false, false};

    /* Variables for Radar communication and controller check */
    uint32_t Radar_last_msg_cnt = 0u;

    /* Variables for GUI communication and data check */
    uint32_t GUI_last_msg_cnt = 0u;

    /* Variables for Lane detection component communication and data check */
    uint32_t LaneDetc_last_msg_cnt = 0u;
    uint16_t LaneDetc_offsetError_cnt = 0u;
    uint16_t LaneDetc_curvatureError_cnt = 0u;

    /* Variables for Object detection component communication and data check */
    uint32_t ObjDetc_last_msg_cnt = 0u;
    uint16_t ObjDetc_noiseError_cnt = 0u;
    bool ObjDetc_last_isObject = false;

    /* Variables for ECU communication and controller check */
    int16_t speed_set = 0u;
    uint32_t ECU_last_msg_cnt = 0u;
    uint16_t ECU_speedError_cnt = 0u;

    /* Variables for Driving mode communication check */
    uint32_t DrivingMode_last_msg_cnt = 0u;

    /* Variables for MPC Control communication check */
    uint32_t MPC_last_msg_cnt = 0u;
};
#endif
