#ifndef DJI_BOARD_CAN_V2_H_
#define DJI_BOARD_CAN_V2_H_

#include "motor_driver.h"
#include "main.h"
#include "BUPTLib_port.h"

#ifdef USE_MTR_DRIVER_DJI_BOARD_V2

//大疆驱动板接收ID基址
#define DJI_BASE_ID (0x200)
/**
  * @brief  电机种类  3508 or 2006
  */
typedef enum {
    RM_3508 = 1,
    M_2006 = 2,
    NONE = 3 //none表示没有接电机
} MotorType_TypeDef;
typedef enum {
    MotorStatus_OFF = 0,
    MotorStatus_POS,
    MotorStatus_VEL,
    MotorStatus_CUR,
    MotorStatus_HOMING,
    MotorStatus_STOP
} RunningStatus;

/**
 *
 * @brief  Robomaster Motor 3508/2006 type structure definition
 * @note
 */
typedef struct {
    int32_t pos, posLast;
    MotorType_TypeDef type;
    int32_t vel; //电机返回的速度，单位： rpm
    int16_t cur; //电机返回的电流值
    int8_t temp; //电机的温度
} MotorType;

typedef struct RM_MotorStatus_t {
    float pos; // rad
    float rpm;
    float current;
    uint8_t Status;
} RM_MotorStatus_t; // 配合东大驱动板使用的状态结构体

extern RM_MotorStatus_t RM_MotorStatus[8];
extern MotorType Motor[8];


/**
 * @brief  CAN消息种类
 */
typedef enum {
    MOTORON = 1,
    MOTOROFF,
    VELCFG,
    POSCFG,
    CURCFG,
    VELCTRL,
    POSCTRL,
    CURCTRL,
    READINFO,
    VELCTRLALL,
    POSCTRLALL,
    HOMING,
    LIMITVELCFG,
    NOINITPOSCFG,
} CANOPTION;


void DJIBoard_MotorOn(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_MotorOff(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_VelCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_LimitVelCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_PosCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int maxPosVel);

void DJIBoard_NoInitPosCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int maxPosVel);

void DJIBoard_CurCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_VelCtrl(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int32_t vel);

void DJIBoard_PosCtrl(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int32_t pos);

void DJIBoard_CurCtrl(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int32_t cur);

void DJIBoard_VelCtrlAll(CAN_HandleTypeDef *hcan, uint32_t boradId, int16_t vel[4]);

void DJIBoard_PosCtrlAll(CAN_HandleTypeDef *hcan, uint32_t boradId, int16_t pos[4]);

void DJIBoard_ReadInfo(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId);

void DJIBoard_Homing(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int16_t vel, int16_t cur);


#endif

#endif