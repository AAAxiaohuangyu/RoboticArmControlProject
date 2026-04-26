#ifndef __ROBOTIC_ARM_CONFIG_H
#define __ROBOTIC_ARM_CONFIG_H

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *Motor_PWM_Timer; // PWM输出的定时器句柄
    uint32_t Motor_PWM_Channel;         // PWM输出的定时器通道
    float Motor_Position;               // 舵机的目标角度,也认为是实际角度,单位为rad
} LFD01M_Motor_Handle_t;

typedef struct
{
    FDCAN_HandleTypeDef *Motor_FDCAN_Handle; //FDCAN通信句柄
    uint16_t Motor_ID;
    uint8_t Motor_Status;
    float Motor_Position_Actual; //单位为rad
    float Motor_Position_Target; // 单位为rad
    float Motor_Velocity_Actual; // 单位为rad/s
    float Motor_Velocity_Target; // 单位为rad/s
    float Motor_Torque_Actual;   //单位为N*m
    float Motor_Torque_Feedforward; //单位为N*m
    float MIT_Kp;
    float MIT_Kd;
} DMJ4310_Motor_Handle_t;

typedef struct
{
    FDCAN_HandleTypeDef *Motor_FDCAN_Handle;
    uint16_t Motor_ID;
    float Motor_Position;//范围:0~2pi,单位为rad
    float Motor_Torque_Current;//范围:-33A~33A,单位为A
} LK4005_Motor_Handle_t;

#define PI 3.14159f
#define LFD01M_Motor_Number 2
#define DMJ4310_Motor_Number 1
#define LK4005_Motor_Number 2

#endif
