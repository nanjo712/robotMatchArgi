#ifndef __WHEELTEC_N100_H
#define __WHEELTEC_N100_H

#include "stdio.h"
#include "stm32f407xx.h"
// #include "sys.h"
#define FRAME_HEADER 0X7B // Frame_header //֡ͷ
#define FRAME_TAIL 0X7D	  // Frame_tail   //֡β
#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11
#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80
#define RS485_RX_DE PAout(11) // 485模式控制.0,接收;1,发送.
#define RS485_RX_RE PAout(12) // 485模式控制.0,接收;1,发送.
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd

#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define IMU_LEN  0x38
#define AHRS_LEN 0x30
typedef struct IMUData_Packet_t
{
	float gyroscope_x;			// unit: rad/s
	float gyroscope_y;			// unit: rad/s
	float gyroscope_z;			// unit: rad/s
	float accelerometer_x;		// m/s^2
	float accelerometer_y;		// m/s^2
	float accelerometer_z;		// m/s^2
	float magnetometer_x;		// mG
	float magnetometer_y;		// mG
	float magnetometer_z;		// mG
	float imu_temperature;		// C
	float Pressure;				// Pa
	float pressure_temperature; // C
	uint32_t Timestamp;			// us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
	float RollSpeed;	// unit: rad/s
	float PitchSpeed;	// unit: rad/s
	float HeadingSpeed; // unit: rad/s
	float Roll;			// unit: rad
	float Pitch;		// unit: rad
	float Heading;		// unit: rad
	float Qw;			// w          //Quaternion
	float Qx;			// x
	float Qy;			// y
	float Qz;			// z
	uint32_t Timestamp; // unit: us
} AHRSData_Packet_t;

typedef enum Data_Type_e
{
	IMU_TYPE,
	AHRS_TYPE
}Data_Type_e;

extern uint8_t ttl_receive;

extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;

// void USART3_SEND(void);
// void USART1_SEND(void);
// void usart2_send(uint8_t data);
// void usart3_send(uint8_t data);
// void usart1_send(uint8_t data);
// void uart1_init(uint32_t bound);
// void uart2_init(uint32_t bound);
// void uart3_init(uint32_t bound);
// void uart5_init(uint32_t bound);

// int USART1_IRQHandler(void);
// int USART2_IRQHandler(void);
// int USART3_IRQHandler(void);
long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);
// void AHRSData2PC(void);
// void IMUData2PC(void);

float XYZ_Target_Speed_transition(uint8_t High, uint8_t Low);
uint8_t TTL_Hex2Dec(uint8_t *data, uint8_t type);
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);

#endif
