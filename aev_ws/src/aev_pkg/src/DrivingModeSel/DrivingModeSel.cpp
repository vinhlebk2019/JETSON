#include "DrivingModeSel.h"

DrivingModeSel::DrivingModeSel(void)
{

}

DrivingModeSel::~DrivingModeSel()
{

}

void DrivingModeSel::DrivingModeSel_MainTask(void)
{
    currentInput = Input;

    if (currentInput.systemReqStop == true && CurrentDrivingMode != StopMode)
    {
        CurrentDrivingMode = EmergecyStop;
    }
    else if (currentInput.systemError == true && CurrentDrivingMode != StopMode)
    {
        CurrentDrivingMode = Deceleration;
    }
    else
    {
        
    }
    
    switch (CurrentDrivingMode)
    {
    case StopMode:
        ROS_INFO("Current mode: StopMode");
        DrivingModeSel_StopModeHdl();
        break;

    case NormalDriving:
        ROS_INFO("Current mode: NormalDriving");
        DrivingModeSel_NormalModeHdl();
        break;

    case LaneChange:
        ROS_INFO("Current mode: LaneChange");
        DrivingModeSel_LaneChangeHdl();
        break;

    case Deceleration:
        ROS_INFO("Current mode: Deceleration");
        DrivingModeSel_DecelerationHdl();
        break;

    case EmergecyStop:
        ROS_INFO("Current mode: EmergecyStop");
        DrivingModeSel_EmergecyHdl();
        break;
            
    default:
        ROS_INFO("Current mode: Invalid");
        break;
    }

    /* Update output */
    Output.drivingMode = CurrentDrivingMode;
}

void DrivingModeSel::DrivingModeSel_StopModeHdl(void)
{
    /* If no obstruction and auto run is enabled, change directly to NormalDriving */
    /* Or wait for user request to run */
    if ((currentInput.Obstruction == false && currentInput.relativeDistance > DISTANCE_HIGH_THRES && currentInput.userReqAutoRun == true)
        || currentInput.userReqStart == true)
    {
        if ((currentInput.systemReqStop == true) || (currentInput.systemError == true)
            || (currentInput.userReqStop == true))
        {

        }
        else
        {
            CurrentDrivingMode = NormalDriving;
        }
        
    }
}

void DrivingModeSel::DrivingModeSel_NormalModeHdl(void)
{
    /* To EmergecyStop Mode */
    if (currentInput.relativeDistance < DISTANCE_LOW_THRES || currentInput.Obstruction == true)
    {
        CurrentDrivingMode = EmergecyStop;
    }
    /* To Parking Mode */
    else if (currentInput.userReqStop == true)
    {
        CurrentDrivingMode = EmergecyStop; //LaneChange;
    }
    /* To Deceleration Mode */
    else if (currentInput.relativeDistance < DISTANCE_HIGH_THRES)
    {
        CurrentDrivingMode = Deceleration;
    }
    else
    {
        /* Keep current mode */
    }
}

void DrivingModeSel::DrivingModeSel_LaneChangeHdl(void)
{
    /* If the car reach the right side of the road and ready to stop
    --> change to Stop mode */
    if (currentInput.centerOffset > LANE_CHANGE_LOW_THRES && currentInput.centerOffset < LANE_CHANGE_HIGH_THRES)
    {
        LaneOffsetAllow_TimeCnt++;
        if (LaneOffsetAllow_TimeCnt >= LANE_CHANGE_ALLOW_TIME_xBASE)
        {
            LaneChangeDone_flag = true;
            LaneOffsetAllow_TimeCnt = 0;
        }
    }
    else
    {
        LaneOffsetAllow_TimeCnt = 0;
        LaneChangeDone_flag = false;
    }

    if (LaneChangeDone_flag == true && currentInput.userReqStop == true)
    {
        CurrentDrivingMode = StopMode;
    }
}

void DrivingModeSel::DrivingModeSel_DecelerationHdl(void)
{
    /* To NormalDriving Mode */
    if (currentInput.relativeDistance > DISTANCE_HIGH_THRES && currentInput.Obstruction == false)
    {
        CurrentDrivingMode = NormalDriving;
    }
    /* To EmergecyStop Mode */
    else if (currentInput.relativeDistance < DISTANCE_LOW_THRES && currentInput.Obstruction == true)
    {
        CurrentDrivingMode = EmergecyStop;
    }
    else
    {
        /* Keep current mode */
    }
}

void DrivingModeSel::DrivingModeSel_EmergecyHdl(void)
{
    if (currentInput.currentSpeed == 0.0f)
    {
        CurrentDrivingMode = StopMode;
    }
    else
    {
        /* Keep current mode */
    }
}
