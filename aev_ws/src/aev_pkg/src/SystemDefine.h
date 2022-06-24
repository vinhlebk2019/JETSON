#ifndef SYSTEM_DEFINE_H
#define SYSTEM_DEFINE_H

#define GUI_MSG_TIME
#define RADAR_MSG_TIME
#define OBJ_DETC_MSG_TIME
#define SYS_MONITOR_MSG_TIME    0.2
#define LANE_DETC_MSG_TIME
#define MPC_MSG_TIME            0.2
#define DRIVING_MODE_MSG_TIME   0.1

union Can_float
{
  float Can_Float_Value;
  struct
  {
    unsigned byte1:8;
    unsigned byte2:8;
    unsigned byte3:8;
    unsigned byte4:8;
  } byte;
};

typedef enum
{
  NoError = 0,

  __ErrorLevel_1_Start,
  SpeedControlError,
  ObjectDetectionNoise,
  __ErrorLevel_1_End,

  __ErrorLevel_2_Start,
  Radar_CommError,
  ECU_CommError,
  GUI_CommError,
  ObjDetc_CommError,
  DrivingMode_CommError,
  __ErrorLevel_2_End,

  __ErrorLevel_3_Start,
  LaneDetc_CommError,
  MPC_CommError,
  MessageTime_CommError,
  CurvatureOutRange,
  LaneFollowingError,
  __ErrorLevel_3_End,

} SystemMonitor_ErrorInfo;

typedef enum
{
  StopMode = 1,
  NormalDriving,
  LaneChange,
  Deceleration,
  EmergecyStop,
} DrivingModeSel_Mode;

#endif