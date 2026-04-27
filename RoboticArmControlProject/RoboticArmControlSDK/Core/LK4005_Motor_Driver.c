#include "LK4005_Motor_Driver.h"

LK4005_Motor_Handle_t LK4005_Motor_Handle[LK4005_Motor_Number]; //0为云台电机,1为小臂电机

void LK4005_Motor_Control_Init(void)
{
    LK4005_Motor_Handle[0].Motor_FDCAN_Handle = &hfdcan1;
    LK4005_Motor_Handle[0].Motor_ID = 0x141;
    LK4005_Motor_Handle[0].Motor_Type = Gimbal;
    LK4005_Motor_Handle[1].Motor_FDCAN_Handle = &hfdcan1;
    LK4005_Motor_Handle[1].Motor_ID = 0x142;
    LK4005_Motor_Handle[1].Motor_Type = Joint_Fore;

    FDCAN_FilterTypeDef sfilter = {0};
    sfilter.IdType = FDCAN_STANDARD_ID;
    sfilter.FilterIndex = 1;
    sfilter.FilterType = FDCAN_FILTER_RANGE;
    sfilter.FilterConfig = FDCAN_RX_FIFO1;
    sfilter.FilterID1 = 0x140;
    sfilter.FilterID2 = 0x14F;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sfilter);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

void LK4005_Motor_Torque_Control(LK4005_Motor_Handle_t LK4005_Motor_Handle)
{
    if (LK4005_Motor_Handle.LK4005_Motor_MIT_Control.Output <= -33.0f)
    {
        LK4005_Motor_Handle.LK4005_Motor_MIT_Control.Output = -33.0f;
    }
    else if (LK4005_Motor_Handle.LK4005_Motor_MIT_Control.Output >= 33.0f)
    {
        LK4005_Motor_Handle.LK4005_Motor_MIT_Control.Output = 33.0f;
    }
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    int16_t Torque_Temp = (int16_t)(LK4005_Motor_Handle.LK4005_Motor_MIT_Control.Output * Torque_Conversion_Ratio);
    FDCAN_Send_Temp[0] = 0xA1;
    FDCAN_Send_Temp[4] = (uint8_t)(Torque_Temp & 0xFF);
    FDCAN_Send_Temp[5] = (uint8_t)((Torque_Temp >> 8) & 0xFF);
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Position_Control(LK4005_Motor_Handle_t LK4005_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    uint32_t Position_Temp = (uint32_t)(LK4005_Motor_Handle.Motor_Position_PID_Control_Handle.Motor_Position_Target *100.0f);
    FDCAN_Send_Temp[0] = 0xA5;
    FDCAN_Send_Temp[1] = LK4005_Motor_Handle.Motor_Position_PID_Control_Handle.Move_Direction;
    FDCAN_Send_Temp[4] = (uint8_t)(Position_Temp & 0xFF);
    FDCAN_Send_Temp[5] = (uint8_t)((Position_Temp >> 8*1) & 0xFF);
    FDCAN_Send_Temp[6] = (uint8_t)((Position_Temp >> 8*2) & 0xFF);
    FDCAN_Send_Temp[7] = (uint8_t)((Position_Temp >> 8*3) & 0xFF);
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Read_Position(LK4005_Motor_Handle_t LK4005_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    FDCAN_Send_Temp[0] = 0x94;
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Response_Data_Explain(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp, uint8_t *FDCAN_Rx_Data_Temp, LK4005_Motor_Handle_t *LK4005_Motor_Handle)
{
    if (hfdcan == LK4005_Motor_Handle->Motor_FDCAN_Handle)
    {
        if (FDCAN_Rx_Head_Temp.Identifier == LK4005_Motor_Handle->Motor_ID && FDCAN_Rx_Head_Temp.DataLength == LK4005_Motor_FDCAN_Length)
        {
            if (FDCAN_Rx_Data_Temp[0] == 0x94)
            {
                LK4005_Motor_Handle->LK4005_Motor_MIT_Control.Motor_Position_Actual = ((float)(((uint32_t)FDCAN_Rx_Data_Temp[7] << 24) | ((uint32_t)FDCAN_Rx_Data_Temp[6] << 16) | ((uint32_t)FDCAN_Rx_Data_Temp[5] << 8) | ((uint32_t)FDCAN_Rx_Data_Temp[4])) * Position_Conversion_Ratio / Reduction_Ratio) * PI / 180.0f;

                LK4005_Motor_Handle->Motor_Position_PID_Control_Handle.Motor_Position_Actual = LK4005_Motor_Handle->LK4005_Motor_MIT_Control.Motor_Position_Actual;
            }
        }
    }
}
