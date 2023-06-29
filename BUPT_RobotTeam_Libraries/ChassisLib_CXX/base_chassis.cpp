/**
 * @file
 * @author LiNY2 
 * @date 2023/03/25 0025
 */



#include "base_chassis.h"
#include "chassis_common_config.h"
#ifdef __cplusplus
extern "C"
{
//CStyle-Link inc,func and var begin
#endif
#include "main.h"
#include "components/vofaDataDriver/vofaDataDriver.h"
#ifdef __cplusplus
}
//CStyle-Link func and var end
#endif
/**
 *
 */
float location_raw_x = 0; ///< 全场定位原始报文 x
float location_raw_y = 0; ///<全场定位原始报文 y
float location_raw_yaw = 1.57 ;///< 全场定位原始报文 yaw;\n 如果不初始化yaw，会导致舵向混乱，表现为大疆电机抽一下
float location_raw_speed_x ;
float location_raw_speed_y ;
float location_raw_omega ;

/**
 * @brief 向串口输出位姿信息
 * @note 在使用日志库的情况下，调用频率不宜太高。
 */
void BaseChassis::Chassis_PrintPostureStatus()
{
    if(!Chassis_PrintPostureStatus_Flag)
    {
        return ;
    }
#if defined(OSLIB_LOG_MODULE_ENABLED)
    log_i("x:%5.3f y:%5.3f yaw:%4.3f vx:%4.3f vy:%4.3f omega:%4.3f",
          postureStatus.x, postureStatus.y,
          postureStatus.yaw,
          postureStatus.speed_x, postureStatus.speed_y,
          postureStatus.omega);
    static float transferPostureStatus[6];
    static char  txbuffer[65];
    static uint16_t msgsize = 0;
    transferPostureStatus[0] = postureStatus.x;
    transferPostureStatus[1] = postureStatus.y;
    transferPostureStatus[2] = postureStatus.yaw;
    transferPostureStatus[3] = postureStatus.speed_x;
    transferPostureStatus[4] = postureStatus.speed_y;
    transferPostureStatus[5] = postureStatus.omega;
    msgsize = vofaDataPut(txbuffer,"pos:",transferPostureStatus,6,0);
//    elog_d("VOFA","%*.*s",msgsize,msgsize,txbuffer);
    OSLIB_UART_SendData(&huart1,(const uint8_t*)txbuffer,msgsize);
#elif defined(OSLIB_UART_MODULE_ENABLED)
    uprintf("--ChassisStatus|x:%5.3f y:%5.3f yaw:%4.3f vx:%4.3f vy:%4.3f omega:%4.3f\r\n",
            postureStatus.x, postureStatus.y,
            postureStatus.yaw,
            postureStatus.speed_x, postureStatus.speed_y,
            postureStatus.omega);
#endif
}

void BaseChassis::Chassis_SetTargetVel(float speed,float dir,float omega,
uint8_t keep_speed,uint8_t keep_dir ,uint8_t keep_omega )
{
    if (!keep_speed)
    {
        BaseChassis::targetVel.speed    = speed ;
    }
    if(!keep_dir)
    {
        BaseChassis::targetVel.dir      = dir ;
    }
    if(!keep_omega)
    {
        BaseChassis::targetVel.omega    = omega ;
    }
}
void BaseChassis::Chassis_SetPosMode(BaseChassis::Chassis_Pos_Mode posMode)
{
    pos_mode = posMode;
}
extern "C"{

void CAN_Callback_Location_ReadPos_X(CAN_ConnMessage *data)
{
//    location_raw_x = data->payload.fl[1];
//    location_raw_speed_x = data->payload.fl[0];
        location_raw_x = data->payload.fl[0];
        location_raw_y = data->payload.fl[1];
        Debug("pos:",location_raw_x,location_raw_y);
}

void CAN_Callback_Location_ReadPos_Y(CAN_ConnMessage *data)
{
//    location_raw_y = data->payload.fl[1];
//    location_raw_speed_y = data->payload.fl[0];
    location_raw_speed_x = data->payload.fl[0];
    location_raw_speed_y = data->payload.fl[1];

}

void CAN_Callback_Location_ReadPos_Yaw(CAN_ConnMessage *data)
{
    float yaw = __ANGLE2RAD(data->payload.fl[1]);
    float temp_yaw = yaw + __ANGLE2RAD(90);
    if (temp_yaw < 0)
    {
        temp_yaw += 2 * PI;
    }
    location_raw_yaw =  temp_yaw;
    location_raw_omega = __ANGLE2RAD(data->payload.fl[0]);
}

}