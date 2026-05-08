#include "Control_Algorithm.h"

float Robotic_Arm_Mass_L1 = 0.6f;
float Robotic_Arm_Mass_L2 = 0.72f;
float Robotic_Arm_Mass_End = 0.1f;

// 弧度归一化到0~2pi
float Normalize_Angle(float Angle)
{
    float Res = fmodf(Angle, 2.0f * PI);
    if (Res < 0)
    {
        Res += 2.0f * PI;
    }
    return Res;
}

/*
//达妙电机由于其特殊的CAN报文控制用不到这个函数
*/
void Motor_MIT_Control(Motor_MIT_Control_Handle_t *Motor_MIT_Control_Handle)
{
    Motor_MIT_Control_Handle->Output = Motor_MIT_Control_Handle->MIT_Kp * (Motor_MIT_Control_Handle->Motor_Position_Target - Motor_MIT_Control_Handle->Motor_Position_Actual) + Motor_MIT_Control_Handle->MIT_Kd * (Motor_MIT_Control_Handle->Motor_Velocity_Target - Motor_MIT_Control_Handle->Motor_Velocity_Actual) + Motor_MIT_Control_Handle->Motor_Torque_Feedforward;

    if (fabsf(Motor_MIT_Control_Handle->Motor_Velocity_Actual) >= 0.001f)
    {
        Motor_MIT_Control_Handle->Output += (Motor_MIT_Control_Handle->Motor_Velocity_Actual) / fabsf(Motor_MIT_Control_Handle->Motor_Velocity_Actual) * Motor_MIT_Control_Handle->Motor_Torque_Friction;
    }
}

float Upperarm_Gravity_Compensation(float Upperarm_Motor_Angle, float Forearm_Motor_Angle)
{
    return ((Robotic_Arm_Mass_L1 * Robotic_Arm_Length_L1 * 0.5f + Robotic_Arm_Mass_Forearm_Motor * Robotic_Arm_Length_L1) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle) + (Robotic_Arm_Mass_L2 * (Robotic_Arm_Length_L2 * 0.5f) + Robotic_Arm_Mass_End * Robotic_Arm_Length_L2) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle + Forearm_Motor_Angle));
}

float Forearm_Gravity_Compensation(float Upperarm_Motor_Angle, float Forearm_Motor_Angle)
{
    return (Robotic_Arm_Mass_L2 * Robotic_Arm_Length_L2 * 0.5f + Robotic_Arm_Mass_End * Robotic_Arm_Length_L2) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle + Forearm_Motor_Angle);
}

void Speed_Plan_Update(Speed_Plan_Handle_t *Speed_Plan_Handle, float position_actual, float position_target)
{
    uint32_t Current_Time = HAL_GetTick();
    float dt = (Current_Time - Speed_Plan_Handle->Time_Stamp) * 0.001f;

    switch (Speed_Plan_Handle->Speed_Plan_State)
    {
    case idle:
    {
        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->v = 0;
        Speed_Plan_Handle->s = 0;
        break;
    }
    case init:
    {
        Speed_Plan_Handle->error_s = position_target - position_actual;
        Speed_Plan_Handle->position_initial = position_actual;

        if (fabsf(Speed_Plan_Handle->error_s) <= Speed_Plan_Handle->Threshold_S)
        {
            Speed_Plan_Handle->Speed_Plan_State = idle;
            break;
        }

        if (Speed_Plan_Handle->error_s >= 0.0f)
        {
            Speed_Plan_Handle->direction_flag = 1.0f;
        }
        else
        {
            Speed_Plan_Handle->direction_flag = -1.0f;
        }
        Speed_Plan_Handle->Speed_Plan_State = phase1;
        break;
    }
    case phase1:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a >= Speed_Plan_Handle->a_max)
        {
            Speed_Plan_Handle->a = Speed_Plan_Handle->a_max;
            Speed_Plan_Handle->s1 = Speed_Plan_Handle->s;
            Speed_Plan_Handle->v1 = Speed_Plan_Handle->v;
            Speed_Plan_Handle->Speed_Plan_State = phase2;
        }
        break;
    }
    case phase2:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v >= Speed_Plan_Handle->v_max - Speed_Plan_Handle->v1)
        {
            Speed_Plan_Handle->s2 = Speed_Plan_Handle->s;
            Speed_Plan_Handle->Speed_Plan_State = phase3;
        }
        break;
    }
    case phase3:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a <= 0)
        {
            Speed_Plan_Handle->a = 0;
            Speed_Plan_Handle->v = Speed_Plan_Handle->v_max;
            Speed_Plan_Handle->s_accel_used = Speed_Plan_Handle->s;
            if (fabsf(Speed_Plan_Handle->error_s) >= powf(Speed_Plan_Handle->v_max, 2.0f) / Speed_Plan_Handle->a_max + Speed_Plan_Handle->a_max * Speed_Plan_Handle->v_max / Speed_Plan_Handle->j)
            {
                Speed_Plan_Handle->Speed_Plan_State = phase4;
            }
            else
            {
                Speed_Plan_Handle->Speed_Plan_State = phase5;
            }
        }
        break;
    }
    case phase4:
    {
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;
        if (Speed_Plan_Handle->s >= (fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s_accel_used))
        {
            Speed_Plan_Handle->Speed_Plan_State = phase5;
        }
        break;
    }
    case phase5:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->s >= (fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s2))
        {
            Speed_Plan_Handle->Speed_Plan_State = phase6;
        }
        break;
    }
    case phase6:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->s >= (fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s1))
        {
            Speed_Plan_Handle->Speed_Plan_State = phase7;
        }
        break;
    }
    case phase7:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v <= 0)
        {
            Speed_Plan_Handle->v = 0;
            Speed_Plan_Handle->Speed_Plan_State = idle;
        }
        break;
    }
    }
    if (fabsf(Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s)) <= 0.003f || Speed_Plan_Handle->v < 0.0f || Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s) >= 0.0f)
    {
        Speed_Plan_Handle->Speed_Plan_State = idle;
    }
    Speed_Plan_Handle->Time_Stamp = HAL_GetTick();
}
