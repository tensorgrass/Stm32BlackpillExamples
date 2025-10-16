/*
 * maincpp.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#include <maincpp.hpp>
#include <UartSerial.hpp>
#include <main.h>

UartSerial uartSerial;

void main_cpp(UART_HandleTypeDef *huart1)
{
	uartSerial.init(huart1);
	while( 1 )
	{
		for (int i = 0; i < 2; i++) {
			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
			HAL_Delay(200);
			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
			HAL_Delay(200);
		}
		HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
		HAL_Delay(200);
	}
}

void HAL_UART_RxCpltCallback_cpp(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) // Verifica que sea el UART correcto
	{
		// Llama al método de la clase UartSerial para manejar la recepción
		uartSerial.receiveData(huart);
	}
}
