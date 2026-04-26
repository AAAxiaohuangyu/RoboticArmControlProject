#include "DMJ4310_Motor_Driver.h"

DMJ4310_Motor_Handle_t DMJ4310_Motor_Handle[DMJ4310_Motor_Number];

void DMJ4310_Motor_Control_Init(void)
{
    DMJ4310_Motor_Handle[0].Motor_FDCAN_Handle = &hfdcan1;
    DMJ4310_Motor_Handle[0].Motor_ID = 1;
    DMJ4310_Motor_Handle[0].MIT_Kd = 0;
    DMJ4310_Motor_Handle[0].MIT_Kp = 0;
    
    FDCAN_FilterTypeDef sfilter = {0};
    sfilter.IdType = FDCAN_STANDARD_ID;
    sfilter.FilterIndex = 0;
    sfilter.FilterType = FDCAN_FILTER_RANGE;
    sfilter.FilterConfig = FDCAN_RX_FIFO0;
    sfilter.FilterID1 = 0x000;
    sfilter.FilterID2 = 0x00F;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sfilter);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/*
MIT控制帧也被视为请求帧,引发DMJ4310发送反馈帧
*/
void DMJ4310_Motor_MIT_Control(DMJ4310_Motor_Handle_t DMJ4310_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[DMJ4310_Motor_FDCAN_Length] = {0};

    uint16_t Position_Temp = Float_To_Uint(DMJ4310_Motor_Handle.Motor_Position_Target, DMJ4310_Motor_Position_Min, DMJ4310_Motor_Position_Max, DMJ4310_Motor_Position_Length);
    uint16_t Velocity_Temp = Float_To_Uint(DMJ4310_Motor_Handle.Motor_Velocity_Target, DMJ4310_Motor_Velocity_Min, DMJ4310_Motor_Velocity_Max, DMJ4310_Motor_Velocity_Length);
    uint16_t MIT_Kp_Temp = Float_To_Uint(DMJ4310_Motor_Handle.MIT_Kp, DMJ4310_Motor_MIT_Kd_Min, DMJ4310_Motor_MIT_Kd_Max, DMJ4310_Motor_MIT_Kd_Length);
    uint16_t MIT_Kd_Temp = Float_To_Uint(DMJ4310_Motor_Handle.MIT_Kd, DMJ4310_Motor_MIT_Kd_Min, DMJ4310_Motor_MIT_Kd_Max, DMJ4310_Motor_MIT_Kd_Length);
    uint16_t Torque_Temp = Float_To_Uint(DMJ4310_Motor_Handle.Motor_Torque_Feedforward, DMJ4310_Motor_Torque_Min, DMJ4310_Motor_Torque_Max, DMJ4310_Motor_Torque_Length);

    FDCAN_Send_Temp[0] = Position_Temp >> 8;
    FDCAN_Send_Temp[1] = Position_Temp & 0xFF;
    FDCAN_Send_Temp[2] = Velocity_Temp >> 4;
    FDCAN_Send_Temp[3] = ((Velocity_Temp & 0xF) << 4) | (MIT_Kp_Temp >> 8);
    FDCAN_Send_Temp[4] = MIT_Kp_Temp & 0xFF;
    FDCAN_Send_Temp[5] = MIT_Kd_Temp >> 4;
    FDCAN_Send_Temp[6] = ((MIT_Kd_Temp & 0xF) << 4) | (Torque_Temp >> 8);
    FDCAN_Send_Temp[7] = Torque_Temp & 0xFF;

    FDCAN_Send_Standard(DMJ4310_Motor_Handle.Motor_FDCAN_Handle, DMJ4310_Motor_Handle.Motor_ID, FDCAN_Send_Temp, DMJ4310_Motor_FDCAN_Length);
}

void DMJ4310_Motor_Enable(DMJ4310_Motor_Handle_t DMJ4310_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[DMJ4310_Motor_FDCAN_Length] = {0};
    FDCAN_Send_Temp[0] = 0xFF;
    FDCAN_Send_Temp[1] = 0xFF;
    FDCAN_Send_Temp[2] = 0xFF;
    FDCAN_Send_Temp[3] = 0xFF;
    FDCAN_Send_Temp[4] = 0xFF;
    FDCAN_Send_Temp[5] = 0xFF;
    FDCAN_Send_Temp[6] = 0xFF;
    FDCAN_Send_Temp[7] = 0xFC;
    FDCAN_Send_Standard(DMJ4310_Motor_Handle.Motor_FDCAN_Handle, DMJ4310_Motor_Handle.Motor_ID, FDCAN_Send_Temp, DMJ4310_Motor_FDCAN_Length);
}

void DMJ4310_Motor_Response_Data_Explain(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp, uint8_t *FDCAN_Rx_Data_Temp, DMJ4310_Motor_Handle_t *DMJ4310_Motor_Handle)
{
    if (hfdcan == DMJ4310_Motor_Handle->Motor_FDCAN_Handle)
    {
        if (FDCAN_Rx_Head_Temp.Identifier == DMJ4310_Motor_FDCAN_Response_ID && FDCAN_Rx_Head_Temp.DataLength == DMJ4310_Motor_FDCAN_Length)
        {
            if ((DMJ4310_Motor_Handle->Motor_ID & 0xFF) == (FDCAN_Rx_Data_Temp[0] >> 4))
            {
                DMJ4310_Motor_Handle->Motor_Status = (FDCAN_Rx_Data_Temp[0] & 0x0F);
                uint16_t Position_Temp = (FDCAN_Rx_Data_Temp[1] << 8) | FDCAN_Rx_Data_Temp[2];
                uint16_t Velocity_Temp = (FDCAN_Rx_Data_Temp[3] << 4) | (FDCAN_Rx_Data_Temp[4] >> 4);
                uint16_t Torque_Temp = ((FDCAN_Rx_Data_Temp[4] & 0x0F) << 8) | FDCAN_Rx_Data_Temp[5];

                DMJ4310_Motor_Handle->Motor_Position_Actual = Uint_To_Float((int)Position_Temp, DMJ4310_Motor_Position_Min, DMJ4310_Motor_Position_Max, DMJ4310_Motor_Position_Length);
                DMJ4310_Motor_Handle->Motor_Velocity_Actual = Uint_To_Float((int)Velocity_Temp, DMJ4310_Motor_Velocity_Min, DMJ4310_Motor_Velocity_Max, DMJ4310_Motor_Velocity_Length);
                DMJ4310_Motor_Handle->Motor_Torque_Actual = Uint_To_Float((int)Torque_Temp, DMJ4310_Motor_Torque_Min, DMJ4310_Motor_Torque_Max, DMJ4310_Motor_Torque_Length);
            }
        }
    }
}
