#include "Robotic_Arm_Communication_HAL_STM32_Port.h"

uint8_t Usart_Used0_Rx_Buff[Usart_Used0_Rx_Buff_Length] = {0};

void Communication_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == Communication_Usart_Instance_Used0)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
    }
}

void Communication_Usart_Used0(void)
{
    uint8_t Usart_Used0_Tx_Buff[Usart_Used0_Tx_Buff_Length] = {0};
    HAL_UART_Transmit_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Tx_Buff, Usart_Used0_Tx_Buff_Length);
    HAL_Delay(Usart_Used0_Transmit_Cycle);
}
