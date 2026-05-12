#include "Robotic_Arm_Communication_HAL_STM32_Port.h"
#include "string.h"
#include "stdio.h"
#include "LFD01M_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "DMJ4310_Motor_Driver.h"
#include "Control_Algorithm.h"

uint8_t Usart_Used0_Rx_Buff[Usart_Used0_Rx_Buff_Length] = {0};

void Communication_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == Communication_Usart_Instance_Used0)
    {
        float X_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[1] << 8) | Usart_Used0_Rx_Buff[0])) / 100.0f;
        float Y_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[3] << 8) | Usart_Used0_Rx_Buff[2])) / 100.0f;
        float Z_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[5] << 8) | Usart_Used0_Rx_Buff[4])) / 100.0f;
        float Servo1_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[7] << 8) | Usart_Used0_Rx_Buff[6]) / 180.0f * PI;
        float Servo2_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[9] << 8) | Usart_Used0_Rx_Buff[8]) / 180.0f * PI;
        Coordinate_Inverse_Settlement(X_Temp,Y_Temp,Z_Temp,&LK4005_Motor_Handle[0].Motor_Position_Target,&DMJ4310_Motor_Handle[0].Motor_Position_Target,&LK4005_Motor_Handle[1].Motor_Position_Target);
        if (DMJ4310_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State == idle && LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State == idle)
        {
            DMJ4310_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
        }
        LFD01M_Motor_Handle[0].Motor_Position = Servo1_Temp;
        LFD01M_Motor_Handle[1].Motor_Position = Servo2_Temp;
        HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
    }
}

void Communication_Test(void)
{
    uint8_t package[40] = {0};

    memcpy(&package[0], &LK4005_Motor_Handle[1].Motor_Position_Target, 4);
    memcpy(&package[4], &LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Position_Actual, 4);
    memcpy(&package[8], &DMJ4310_Motor_Handle[0].Motor_Position_Target, 4);
    memcpy(&package[12], &DMJ4310_Motor_Handle[0].Motor_MIT_Control_Handle.Motor_Position_Actual, 4);
    memcpy(&package[16], &LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Output, 4);
    memcpy(&package[20], &LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Position_Target, 4);
    memcpy(&package[24], &LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Velocity_Target, 4);
    memcpy(&package[28], &LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Torque_Feedforward, 4);
    memcpy(&package[32], &DMJ4310_Motor_Handle[0].Motor_MIT_Control_Handle.Motor_Torque_Feedforward, 4);
    package[36] = 0x00;
    package[37] = 0x00;
    package[38] = 0x80;
    package[39] = 0x7f;
    HAL_UART_Transmit_DMA(&huart1, package, 40);
    HAL_Delay(3);
}
