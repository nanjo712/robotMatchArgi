/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifdef __cplusplus
extern "C" {
#endif
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef __cplusplus
};
#endif

#define LOG_TAG    "defTask"

#ifdef __cplusplus
extern "C"{
#endif

#include "oslib.h"
#include "wheeltec_N100.h"

#ifdef __cplusplus
}
#endif
#include "ChassisLib_CXX/rubberwheel_chassis.h"
#include "ChassisLib_CXX/ctrlGo2Point.h"
#include <cstring>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuDataRead */
osThreadId_t imuDataReadHandle;
const osThreadAttr_t imuDataRead_attributes = {
  .name = "imuDataRead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uart3Semaphores */
osSemaphoreId_t uart3SemaphoresHandle;
const osSemaphoreAttr_t uart3Semaphores_attributes = {
  .name = "uart3Semaphores"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#ifdef __cplusplus
extern "C"{
#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartImuDataRead(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    OSLIB_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart3Semaphores */
  uart3SemaphoresHandle = osSemaphoreNew(1, 1, &uart3Semaphores_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imuDataRead */
  imuDataReadHandle = osThreadNew(StartImuDataRead, NULL, &imuDataRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
//    rubberWheel_chassis *chassis = new rubberWheel_chassis();
//    chassis->ChassisInit();
//    Go2Point *p = new Go2Point(chassis);
//  p->SetTarget(Point2D_s{2,2},)
    /* Infinite loop */
    for(;;)
    {
//        chassis->ChassisCtrlVel(1);
//      chassis->ChassisCtrlRot(1);
        HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
        osDelay(500);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartImuDataRead */
/**
* @brief Function implementing the imuDataRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuDataRead */
void StartImuDataRead(void *argument)
{
  /* USER CODE BEGIN StartImuDataRead */
    OSLIB_UART_Handle_t *uart_handle = OSLIB_UART_Handle_Get(&huart2);
    uint8_t FD_Data[80],cnt=0;
    uint32_t IMUlastStamp=0,AHRSlastStamp=0;
    /* Infinite loop */
    for(;;)
    {
        cnt=0;
        osMessageQueueGet(uart_handle->rx.it.rx_queue,FD_Data+cnt,NULL,osWaitForever);
        while (FD_Data[0]!=FRAME_HEAD) osMessageQueueGet(uart_handle->rx.it.rx_queue,FD_Data+cnt,NULL,osWaitForever);
        cnt=1;
        for (;;)
        {
            osMessageQueueGet(uart_handle->rx.it.rx_queue, FD_Data + cnt, NULL, osWaitForever);
            if (FD_Data[cnt] == FRAME_END && FD_Data[0] == FRAME_HEAD)
            {
                if (FD_Data[1] == TYPE_IMU)
                {
                    uint8_t tag = TTL_Hex2Dec(FD_Data, IMU_TYPE);
                    if (tag==1 && IMUData_Packet.Timestamp - IMUlastStamp >= 5000000)
                    {
                        uprintf("accelerometer_x : %f\n", IMUData_Packet.accelerometer_x);
                        uprintf("accelerometer_y : %f\n", IMUData_Packet.accelerometer_y);
                        uprintf("accelerometer_z : %f\n", IMUData_Packet.accelerometer_z);
                        IMUlastStamp = IMUData_Packet.Timestamp;
                    }
                }
                else if (FD_Data[1] == TYPE_AHRS)
                {
                    uint8_t tag = TTL_Hex2Dec(FD_Data, AHRS_TYPE);
                    if (tag==1 && AHRSData_Packet.Timestamp - AHRSlastStamp >= 5000000)
                    {
                        uprintf("Roll    : %f\n", AHRSData_Packet.Roll);
                        uprintf("Pitch   : %f\n", AHRSData_Packet.Pitch);
                        uprintf("Heading : %f\n", AHRSData_Packet.Heading);
                        AHRSlastStamp = AHRSData_Packet.Timestamp;
                    }
                }
                memset(FD_Data,0,sizeof(FD_Data));
                break;
            }
            cnt++;
        }
    }
  /* USER CODE END StartImuDataRead */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#ifdef __cplusplus
}
#endif
/* USER CODE END Application */

