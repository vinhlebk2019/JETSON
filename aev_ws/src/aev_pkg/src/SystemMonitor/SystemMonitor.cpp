#include "SystemMonitor.h"

SystemMonitor::SystemMonitor(void)
{
    SystemMonitor_RefreshError();
}

SystemMonitor::~SystemMonitor()
{

}

void SystemMonitor::SystemMonitor_Radar_MsgCheck(uint32_t msg_cnt)
{
    if (Radar_last_msg_cnt == msg_cnt)
    {
        Output.ErrorInfo = Radar_CommError;
    }
    else
    {

    }
    Radar_last_msg_cnt = msg_cnt;

    new_msg_flag[Radar_MsgCheck] = true;
    msg_time_xbase_thres[Radar_MsgCheck] = 15u;
}

void SystemMonitor::SystemMonitor_ECU_MsgCheck(uint32_t msg_cnt, float current_speed)
{
    if (ECU_last_msg_cnt == msg_cnt)
    {
        Output.ErrorInfo = ECU_CommError;
        LaneDetc_offsetError_cnt = 0;
    }
    else
    {
        /* If actual speed out of deviation range in a definition time */
        /* Then the error shall raise for later action */
        if (((current_speed > speed_set) &&  ((current_speed - speed_set) > SPEED_DEVIATION))
        || ((speed_set > current_speed) &&  ((speed_set - current_speed) > SPEED_DEVIATION)))
        {
            ECU_speedError_cnt++;
        }
        else
        {
            ECU_speedError_cnt = 0;
        }
        if (ECU_speedError_cnt > SPEED_CONTROL_ERROR_TIME_CNT)
        {
            Output.ErrorInfo = SpeedControlError;
            ECU_speedError_cnt = 0;
        }
        
    }
    ECU_last_msg_cnt = msg_cnt;

    new_msg_flag[ECU_MsgCheck] = true;
    msg_time_xbase_thres[ECU_MsgCheck] = 15u;
}

void SystemMonitor::SystemMonitor_GUI_MsgCheck(uint32_t msg_cnt, int16_t speed_setpoint)
{
    if (GUI_last_msg_cnt == msg_cnt)
    {
        Output.ErrorInfo = GUI_CommError;
    }
    else
    {
        speed_set = speed_setpoint;
    }
    GUI_last_msg_cnt = msg_cnt;

    new_msg_flag[GUI_MsgCheck] = true;
    msg_time_xbase_thres[GUI_MsgCheck] = 15u;
}

void SystemMonitor::SystemMonitor_LaneDetc_MsgCheck(uint32_t msg_cnt, float center_offset, float curvature)
{
    /* Message counter must be diff between two time checking */
    if (LaneDetc_last_msg_cnt == msg_cnt)
    {
        //Output.ErrorInfo = LaneDetc_CommError;
        LaneDetc_offsetError_cnt = 0;
        LaneDetc_curvatureError_cnt = 0;
    }
    else
    {
        if (center_offset > LANE_OFFSET_MAX_THRES || center_offset < -(float)LANE_OFFSET_MAX_THRES)
        {
            //Output.ErrorInfo = LaneFollowingError;
            LaneDetc_offsetError_cnt = 0;
        }
        else
        {
            /* Center offset higher than thres within a definition time, raise the error */
            if (center_offset > LANE_OFFSET_THRES || center_offset < -(float)LANE_OFFSET_THRES)
            {
                LaneDetc_offsetError_cnt++;
            }
            else
            {
                LaneDetc_offsetError_cnt = 0;
            }
            if (LaneDetc_offsetError_cnt > CENTER_OFFSET_ERROR_TIME_CNT)
            {
                //Output.ErrorInfo = LaneFollowingError;
                LaneDetc_offsetError_cnt = 0;
            }
        }

        /* curvature lower than thres within a definition time, raise the error */
        if (curvature < CURVATURE_ERROR_THRES)
        {
            LaneDetc_curvatureError_cnt++;
        }
        else
        {
            LaneDetc_curvatureError_cnt = 0;
        }
        if (LaneDetc_curvatureError_cnt > CURVATURE_ERROR_TIME_CNT)
        {
            //Output.ErrorInfo = CurvatureOutRange;
            LaneDetc_curvatureError_cnt = 0;
        }
    }
    LaneDetc_last_msg_cnt = msg_cnt;

    new_msg_flag[LaneDetc_MsgCheck] = true;
    msg_time_xbase_thres[LaneDetc_MsgCheck] = 30u;
}

void SystemMonitor::SystemMonitor_ObjDetc_MsgCheck(uint32_t msg_cnt, bool isObject)
{
    /* Message counter must be diff between two time checking */
    if (ObjDetc_last_msg_cnt == msg_cnt)
    {
        Output.ErrorInfo = ObjDetc_CommError;
    }
    else
    {
        /* If the result of object detection changed continously in a definition time */
        /* Then it is not a good result --> raise error */
        if (ObjDetc_last_isObject != isObject)
        {
            ObjDetc_noiseError_cnt++;
        }
        else
        {
            ObjDetc_noiseError_cnt = 0;
        }

        if (ObjDetc_noiseError_cnt > OBJECT_DETC_NOISE_ERROR_CNT)
        {
            Output.ErrorInfo = ObjectDetectionNoise;
            ObjDetc_noiseError_cnt = 0;
        }
        ObjDetc_last_isObject = isObject;
    }
    ObjDetc_last_msg_cnt = msg_cnt;

    new_msg_flag[ObjDetc_MsgCheck] = true;
    msg_time_xbase_thres[ObjDetc_MsgCheck] = 15u;
}

void SystemMonitor::SystemMonitor_DrivingMode_MsgCheck(uint32_t msg_cnt)
{
    if (DrivingMode_last_msg_cnt == msg_cnt)
    {
        Output.ErrorInfo = DrivingMode_CommError;
    }
    else
    {

    }
    DrivingMode_last_msg_cnt = msg_cnt;

    new_msg_flag[DrivingMode_MsgCheck] = true;
    msg_time_xbase_thres[DrivingMode_MsgCheck] = 15u;
}

void SystemMonitor::SystemMonitor_MPC_MsgCheck(uint32_t msg_cnt)
{
    if (MPC_last_msg_cnt == msg_cnt)
    {
        Output.ErrorInfo = MPC_CommError;
    }
    else
    {

    }
    MPC_last_msg_cnt = msg_cnt;

    new_msg_flag[MPC_MsgCheck] = true;
    msg_time_xbase_thres[MPC_MsgCheck] = 15u;
}

void SystemMonitor::SystemMonitor_MsgTimeCheck(void)
{
    uint8_t inx = 0u;
    for (inx = 0u; inx < msg_type_last; inx++)
    {
        if (new_msg_flag[inx] == false)
        {
            msg_time_xbase[inx] ++;
        }
        else
        {
            new_msg_flag[inx] = false;
            msg_time_xbase[inx] = 0u;
        }

        if (msg_time_xbase[inx] > msg_time_xbase_thres[inx])
        {
            Output.ErrorInfo = MessageTime_CommError;
        }
    }
}

void SystemMonitor::SystemMonitor_RefreshError(void)
{
    Output.Error_flag = false;
    Output.StopRequest_flag = false;
    Output.ErrorInfo = NoError;
    uint8_t inx = 0u;
    for (inx = 0u; inx < msg_type_last; inx++)
    {
        msg_time_xbase[inx] = 0;
        new_msg_flag[inx] = false;
    }
}

void SystemMonitor::SystemMonitor_SystemCheck(void)
{
    SystemMonitor_MsgTimeCheck();

    /* Check the error level in system */
    if (Output.ErrorInfo > __ErrorLevel_1_Start && Output.ErrorInfo < __ErrorLevel_1_End)
    {
        Output.Error_flag = true;
    }
    else if (Output.ErrorInfo > __ErrorLevel_2_Start && Output.ErrorInfo < __ErrorLevel_2_End)
    {
        Output.Error_flag = true;
    }
    else if (Output.ErrorInfo > __ErrorLevel_3_Start && Output.ErrorInfo < __ErrorLevel_3_End)
    {
        Output.Error_flag = true;
        Output.StopRequest_flag = true;
    }
    else
    {
        /* Keepp the output until next action */
    }
}
