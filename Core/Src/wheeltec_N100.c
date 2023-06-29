

#include "wheeltec_N100.h"
#include "math.h"
#include "usart.h"
//#include "cmd.h"


/**
 * @brief 将接收到的消息数组转换填入结构体
 *
 * @param data 数组指针
 * @param type 传入数据类型（IMU_TYPE/AHRS_TYPE）
 * @return uint8_t 0失败，1成功
 */

IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;



uint8_t TTL_Hex2Dec(uint8_t *data, uint8_t type)
{
    if (type == IMU_TYPE)
    {
        if (data[1] == TYPE_IMU && data[2] == IMU_LEN)
        {
            IMUData_Packet.gyroscope_x = DATA_Trans(data[7], data[8], data[9], data[10]); //角速度
            IMUData_Packet.gyroscope_y = DATA_Trans(data[11], data[12], data[13], data[14]);
            IMUData_Packet.gyroscope_z = DATA_Trans(data[15], data[16], data[17], data[18]);

            IMUData_Packet.accelerometer_x = DATA_Trans(data[19], data[20], data[21], data[22]); //线加速度
            IMUData_Packet.accelerometer_y = DATA_Trans(data[23], data[24], data[25], data[26]);
            IMUData_Packet.accelerometer_z = DATA_Trans(data[27], data[28], data[29], data[30]);

            IMUData_Packet.magnetometer_x = DATA_Trans(data[31], data[32], data[33], data[34]); //磁力计数据
            IMUData_Packet.magnetometer_y = DATA_Trans(data[35], data[36], data[37], data[38]);
            IMUData_Packet.magnetometer_z = DATA_Trans(data[39], data[40], data[41], data[42]);

            IMUData_Packet.Timestamp = timestamp(data[55], data[56], data[57], data[58]); //时间戳
                                                                                          // IMUData2PC();
        }
        else
        {
            return 0;
        }
    }
    if (type == AHRS_TYPE)
    {
        if (data[1] == TYPE_AHRS && data[2] == AHRS_LEN)
        {
            AHRSData_Packet.RollSpeed = DATA_Trans(data[7], data[8], data[9], data[10]);       //横滚角速度
            AHRSData_Packet.PitchSpeed = DATA_Trans(data[11], data[12], data[13], data[14]);   //俯仰角速度
            AHRSData_Packet.HeadingSpeed = DATA_Trans(data[15], data[16], data[17], data[18]); //偏航角速度

            AHRSData_Packet.Roll = DATA_Trans(data[19], data[20], data[21], data[22]);    //横滚角
            AHRSData_Packet.Pitch = DATA_Trans(data[23], data[24], data[25], data[26]);   //俯仰角
            AHRSData_Packet.Heading = DATA_Trans(data[27], data[28], data[29], data[30]); //偏航角

            // uprintf("%f\n",AHRSData_Packet.Pitch);

            AHRSData_Packet.Qw = DATA_Trans(data[31], data[32], data[33], data[34]); //四元数
            AHRSData_Packet.Qx = DATA_Trans(data[35], data[36], data[37], data[38]);
            AHRSData_Packet.Qy = DATA_Trans(data[39], data[40], data[41], data[42]);
            AHRSData_Packet.Qz = DATA_Trans(data[43], data[44], data[45], data[46]);
            AHRSData_Packet.Timestamp = timestamp(data[47], data[48], data[49], data[50]); //时间戳                                                                           // AHRSData2PC();
        }
        else
        {
            return 0;
        }
    }
    return 1;
}

float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    long long transition_32;
    float tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;
    transition_32 = 0;
    transition_32 |= Data_4 << 24;
    transition_32 |= Data_3 << 16;
    transition_32 |= Data_2 << 8;
    transition_32 |= Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1; //符号位
    //先右移操作，再按位与计算，出来结果是30到23位对应的e
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    //将22~0转化为10进制，得到对应的x系数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);
    return tmp;
}

long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
  uint32_t transition_32;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
	return transition_32;
}

