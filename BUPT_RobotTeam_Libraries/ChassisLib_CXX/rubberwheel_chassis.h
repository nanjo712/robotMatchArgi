//
// Created by woshiren on 2023/6/24.
//

#ifndef TEST_RUBBERWHEEL_CHASSIS_H
#define TEST_RUBBERWHEEL_CHASSIS_H

#include "base_chassis.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
#include "BUPTLib_port.h"
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
#ifdef __cplusplus
}
#endif


class rubberWheel_chassis : public BaseChassis
{
public:
    rubberWheel_chassis();
    ~rubberWheel_chassis() override;
    void Chassis_UpdatePostureStatus();
    /**
     * @brief 四轮差速底盘运动解算
     * @param speed 速度大小(m/s)
     * @param omega 自转角速度(rad/s)
     * @note 所有参数均为车体坐标系，取车体中心为原点，车头方向为x轴，速度方向取向前为正，角速度方向取逆时针为正
     **/
    void ChassisMove(float target_speed, float target_dir, float target_omega) override;
    void ChassisCtrlVel(float target_speed);
    void ChassisCtrlRot(float target_omega);

    /**
     * @brief 电机要延迟初始化，不能和对象一起初始化
     */
    void ChassisInit() override final;
protected:

};


#endif //TEST_RUBBERWHEEL_CHASSIS_H
