#include "Control_Algorithm.h"

/*
//达妙电机由于其特殊的CAN报文控制用不到这个函数
*/
void Motor_MIT_Control(Motor_MIT_Control_Handle_t *Motor_MIT_Control_Handle)
{
    Motor_MIT_Control_Handle->Output = Motor_MIT_Control_Handle->Output_Conversion_Ratio * (Motor_MIT_Control_Handle->MIT_Kp * (Motor_MIT_Control_Handle->Motor_Position_Target - Motor_MIT_Control_Handle->Motor_Position_Actual) + Motor_MIT_Control_Handle->MIT_Kd * (Motor_MIT_Control_Handle->Motor_Velocity_Target - Motor_MIT_Control_Handle->Motor_Velocity_Actual) + Motor_MIT_Control_Handle->Motor_Torque_Feedforward);
}

float Upperarm_Gravity_Compensation(float Upperarm_Motor_Angle, float Forearm_Motor_Angle)
{
    return ((Robotic_Arm_Mass_L1 * Robotic_Arm_Length_L1 * 0.5f + (Robotic_Arm_Mass_Forearm_Motor + Robotic_Arm_Mass_L2 + Robotic_Arm_Mass_End) * Robotic_Arm_Length_L1) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle) + (Robotic_Arm_Mass_L2 * Robotic_Arm_Length_L2 * 0.5f + Robotic_Arm_Mass_End * Robotic_Arm_Length_L2) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle + Forearm_Motor_Angle));
}

float Forearm_Gravity_Compensation(float Upperarm_Motor_Angle, float Forearm_Motor_Angle)
{
    return (Robotic_Arm_Mass_L2 * Robotic_Arm_Length_L2 * 0.5f + Robotic_Arm_Mass_End * Robotic_Arm_Length_L2) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle + Forearm_Motor_Angle);
}
