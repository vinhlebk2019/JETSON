#ifndef DRIVING_MODE_H
#define DRIVING_MODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../SystemDefine.h"

/* User define parameters */

#define DISTANCE_HIGH_THRES 6.0 /* meter */
#define DISTANCE_LOW_THRES 3.0 /* meter */

#define LANE_CHANGE_HIGH_THRES 0.3 /* meter */
#define LANE_CHANGE_LOW_THRES 0.3 /* meter */
#define LANE_CHANGE_ALLOW_TIME_xBASE 10 /* meter */

/* Output of DrivingModeSel */
typedef struct
{
    DrivingModeSel_Mode drivingMode = StopMode;
} DrivingModeSel_Output;

/* Inputr required */
typedef struct
{
    // Gui data
    bool userReqStart = false;
    bool userReqStop = false;
    bool userReqAutoRun = false;

    // System monitor data
    bool systemReqStop = false;
    bool systemError = false;

    // Object detection data
    bool Obstruction = false;

    // Lane detection data
    float centerOffset = 0.0f;

    // Stm32 board data
    float currentSpeed = 0.0f;

    // Radar data
    float relativeDistance = 10.0f;
} DrivingModeSel_Input;

class DrivingModeSel
{
    public:
        DrivingModeSel(void);
        // explicit
        DrivingModeSel(DrivingModeSel &&) {}
        // implicit
        DrivingModeSel(const DrivingModeSel&) = default;
        DrivingModeSel& operator=(const DrivingModeSel&) = default;
        ~DrivingModeSel();

        DrivingModeSel_Output Output;
        DrivingModeSel_Input Input;

        /* Main function of DrivingModeSel, run cyclically */
        void DrivingModeSel_MainTask(void);

    private:
        DrivingModeSel_Input currentInput;
        DrivingModeSel_Mode CurrentDrivingMode = StopMode;

        int LaneOffsetAllow_TimeCnt = 0;
        bool LaneChangeDone_flag = false;

        /* FUntion handler for each mode */
        void DrivingModeSel_StopModeHdl(void);
        void DrivingModeSel_NormalModeHdl(void);
        void DrivingModeSel_LaneChangeHdl(void);
        void DrivingModeSel_DecelerationHdl(void);
        void DrivingModeSel_EmergecyHdl(void);

};
#endif
