/*
 * putchar.c
 *
 *  Created on: Jan 29, 2023
 *      Author: Michal Szozdowski
 */

#include "main.h"
#include "putchar.h"
#include "usart.h"

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart1, &ch2, 1, 1000);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
    return 1;
}
