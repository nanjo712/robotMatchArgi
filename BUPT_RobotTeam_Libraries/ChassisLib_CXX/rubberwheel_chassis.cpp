//
// Created by woshiren on 2023/6/24.
//

#include "rubberwheel_chassis.h"

#define USE_MTR_DRIVER_DJI_BOARD_V2

#ifdef __cplusplus
extern "C"
{
#endif
///< CStyle-Link inc,func and var begin
#include "BUPTLib_port.h"
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
#include "MotorLib/dji_boardv2_can.h"

//#include "point.h"

#ifdef __cplusplus
}
#endif

extern float location_raw_x ; ///< 全场定位原始报文 x
extern float location_raw_y ; ///<全场定位原始报文 y
extern float location_raw_yaw ; ///< 全场定位原始报文 yaw;\n 如果不初始化yaw，会导致舵向混乱，表现为大疆电机抽一下
extern float location_raw_speed_x ;
extern float location_raw_speed_y ;
extern float location_raw_omega ;

rubberWheel_chassis::rubberWheel_chassis()
{

}
rubberWheel_chassis::~rubberWheel_chassis()
{

}

// 初始化底盘电机
void rubberWheel_chassis::ChassisInit()
{
    for (int i = 0; i < 4; i++)
    {
        DJIBoard_MotorOn(&hcan1,DJIV2_BOARDID,i);  // 使能电机
        DJIBoard_VelCfg(&hcan1,DJIV2_BOARDID,i);  // 调整为速度环控制模式
    }
}

/**
  * @brief 四轮差速底盘位姿信息更新
  * @note  此函数调用频率不应低于控制频率
  */
void rubberWheel_chassis::Chassis_UpdatePostureStatus()
{
    postureStatus.x = + location_raw_y + postureStatus.pos_corr_x;
    postureStatus.y = - location_raw_x + postureStatus.pos_corr_y;
    postureStatus.speed_x = + location_raw_speed_y ;
    postureStatus.speed_y = - location_raw_speed_x ;
    // x,y相反是因为坐标系定义问题，全场定位报文的坐标系与底盘位姿的坐标系差了一个90°
    // 即初始时刻车头朝向世界坐标系的正方向

    postureStatus.yaw = location_raw_yaw;
    postureStatus.omega = location_raw_omega;
    //z轴指向一致，不需要换算
}

/**  传入参数按照车体坐标系下的速度、角度和角速度处理
  *  值得注意的是，对于此类底盘，角度只能是0°和180°，
  *  可以用速度的正负来表示，故target_dir事实上是无效参数，
  *  为了保持与基类的兼容，未作删除
  *   by woshiren
  */
void rubberWheel_chassis::ChassisMove(float target_speed, float target_dir, float target_omega)
{
    if (target_speed > DRIVE_WHEEL_MAX_SPEED)
    {
        target_speed = DRIVE_WHEEL_MAX_SPEED;
    }
    else if (target_speed < -DRIVE_WHEEL_MAX_SPEED)
    {
        target_speed = -DRIVE_WHEEL_MAX_SPEED;
    }
    //限制速度大小

    float driveWheelSpeed[4];
    driveWheelSpeed[0]=target_speed  // 线速度解算
            - target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算
    driveWheelSpeed[1]=target_speed  // 线速度解算
            - target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算
    driveWheelSpeed[2]=target_speed  // 线速度解算
            + target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算
    driveWheelSpeed[3]=target_speed  // 线速度解算
            + target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算

    // 传入指令
    for (uint8_t i = 0 ; i < 4 ; i++) {
        DJIBoard_VelCtrl(&hcan1, DJIV2_BOARDID, i + 1, driveWheelSpeed[i] / DRIVE_WHEEL_RADIUS * 10);
        osDelay(1);
    }
}

//直线运动控制
void rubberWheel_chassis::ChassisCtrlVel(float target_speed)
{
    ChassisMove(target_speed,0,0);
}

//旋转运动控制
void rubberWheel_chassis::ChassisCtrlRot(float target_omega)
{
    ChassisMove(0,0,target_omega);
}


