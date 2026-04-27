#include "Robotic_Arm_Control_API.h"

void LFD01M_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    for (i = 0; i < LFD01M_Motor_Number;i++)
    {
        if (LFD01M_Motor_Handle[i].Motor_Type == Servo)
        {
            LFD01M_Motor_Set_Angle(LFD01M_Motor_Handle[i]);
        }
        HAL_Delay(LFD01M_Motor_Control_Cycle);
    }
}

void DMJ4310_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    float Motor_Angle_Temp = 0;
    for (i = 0; i < LK4005_Motor_Number; i++)
    {
        if (LK4005_Motor_Handle[i].Motor_Type == Joint_Fore)
        {
            Motor_Angle_Temp = LK4005_Motor_Handle[i].LK4005_Motor_MIT_Control.Motor_Position_Actual;
            break;
        }
    }
    for (i = 0; i < DMJ4310_Motor_Number; i++)
    {
        if (DMJ4310_Motor_Handle[i].Motor_Type == Joint_Upper)
        {
            DMJ4310_Motor_Handle[i].DMJ4310_Motor_MIT_Control.Motor_Torque_Feedforward = Upperarm_Gravity_Compensation(DMJ4310_Motor_Handle[i].DMJ4310_Motor_MIT_Control.Motor_Position_Actual, Motor_Angle_Temp);
            DMJ4310_Motor_MIT_Control(DMJ4310_Motor_Handle[i]);
        }
        HAL_Delay(DMJ4310_Motor_Control_Cycle);
    }
}

void LK4005_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    float Motor_Angle_Temp = 0;
    for (i = 0; i < DMJ4310_Motor_Number; i++)
    {
        if (DMJ4310_Motor_Handle[i].Motor_Type == Joint_Upper)
        {
            Motor_Angle_Temp = DMJ4310_Motor_Handle[i].DMJ4310_Motor_MIT_Control.Motor_Position_Actual;
            break;
        }
    }
    for (i = 0; i < LK4005_Motor_Number; i++)
    {
        LK4005_Motor_Read_Position(LK4005_Motor_Handle[i]);
        HAL_Delay(2); //微小延迟完成FDCAN报文发送

        if (LK4005_Motor_Handle[i].Motor_Type == Joint_Fore)
        {
            LK4005_Motor_Handle[i].LK4005_Motor_MIT_Control.Motor_Torque_Feedforward = Forearm_Gravity_Compensation(Motor_Angle_Temp,LK4005_Motor_Handle[i].LK4005_Motor_MIT_Control.Motor_Position_Actual);
            Motor_MIT_Control(&LK4005_Motor_Handle[i].LK4005_Motor_MIT_Control);
            LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i]);
        }

        if (LK4005_Motor_Handle[i].Motor_Type == Gimbal)
        {
            LK4005_Motor_Position_Control(LK4005_Motor_Handle[i]);
        }

        HAL_Delay(LK4005_Motor_Control_Cycle);
    }
}

void Robotic_Arm_Control_Init(void)
{
    Motor_Control_Init();
    HAL_Delay(10);
    uint8_t i = 0;
    for (i = 0; i < DMJ4310_Motor_Number;i++)
    {
        DMJ4310_Motor_Enable(DMJ4310_Motor_Handle[i]);
    }
    Communication_Usart_Init();
}

void Robotic_Arm_Control(void)
{
    LFD01M_Motor_Handle_Update();
    DMJ4310_Motor_Handle_Update();
    LK4005_Motor_Handle_Update();
}

void Robotic_Arm_Communication(void)
{
    Communication_Usart_Used0();
}
