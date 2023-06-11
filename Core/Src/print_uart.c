/*
 * print_uart.c
 *
 *  Created on: Apr 12, 2023
 *      Author: Kacper
 */
#include "usart.h"


/* Funkcja która pozwala na wysyłanie danych przez uart za pomocą printf*/
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 1000);
	return ch;
}

