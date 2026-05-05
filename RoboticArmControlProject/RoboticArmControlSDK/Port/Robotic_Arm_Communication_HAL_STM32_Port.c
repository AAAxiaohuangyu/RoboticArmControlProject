#include "Robotic_Arm_Communication_HAL_STM32_Port.h"
#include "string.h"
#include "stdio.h"
#include "LFD01M_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "DMJ4310_Motor_Driver.h"

uint8_t Usart_Used0_Rx_Buff[Usart_Used0_Rx_Buff_Length] = {0};

void Communication_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == Communication_Usart_Instance_Used0)
    {
        Usart_Used0_Rx_Buff[Size] = '\0';
        char tittle[8] = {0};
        int subtitle = 0;
        float temp1 = 0;
        if (strncmp((char *)Usart_Used0_Rx_Buff, "LFD", 3) == 0)
        {
            sscanf((char *)Usart_Used0_Rx_Buff, "%s %d %f", tittle,&subtitle,&temp1);

            if(subtitle == 0)
            {
                LFD01M_Motor_Handle[0].Motor_Position = temp1;
            }
            else if(subtitle == 1)
            {
                LFD01M_Motor_Handle[1].Motor_Position = temp1;
            }
        }

        else if (strncmp((char *)Usart_Used0_Rx_Buff, "LK", 2) == 0)
        {
            sscanf((char *)Usart_Used0_Rx_Buff, "%s %d %f", tittle, &subtitle, &temp1);

            if (subtitle == 0)
            {
                LK4005_Motor_Handle[0].Motor_Position_Target = temp1;
            }
            else if (subtitle == 1)
            {
                LK4005_Motor_Handle[1].Motor_Position_Target = temp1;
                LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            }
        }

        else if (strncmp((char *)Usart_Used0_Rx_Buff, "DM", 2) == 0)
        {
            sscanf((char *)Usart_Used0_Rx_Buff, "%s %f", tittle, &temp1);
            DMJ4310_Motor_Handle[0].Motor_Position_Target = temp1;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
    }
}

void Communication_Test(void)
{
    uint8_t package[24] = {0};

    memcpy(&package[0], &LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.s, 4);
    memcpy(&package[4], &LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.v, 4);
    memcpy(&package[8], &LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.a, 4);
    memcpy(&package[12], &LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Target, 4);
    memcpy(&package[16], &LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual, 4);
    package[20] = 0x00;
    package[21] = 0x00;
    package[22] = 0x80;
    package[23] = 0x7f;
    HAL_UART_Transmit_DMA(&huart1, package, 24);
    HAL_Delay(3);
}

void Communication_Usart_Used0(void)
{
    uint8_t Usart_Used0_Tx_Buff[Usart_Used0_Tx_Buff_Length] = {0};
    HAL_UART_Transmit_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Tx_Buff, Usart_Used0_Tx_Buff_Length);
    HAL_Delay(Usart_Used0_Transmit_Cycle);
}
