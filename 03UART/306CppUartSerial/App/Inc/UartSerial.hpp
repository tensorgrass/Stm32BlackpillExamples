/*
 * UartSerial.h
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#ifndef UARTSERIAL_H_
#define UARTSERIAL_H_

#include <stm32f4xx_hal.h>

class UartSerial {
public:
	UartSerial();
	virtual ~UartSerial();
	virtual void init(UART_HandleTypeDef *huart);
	virtual void receiveData(UART_HandleTypeDef *huart);

private:
	#define RX_SERIAL_BUFFER_SIZE 100
	uint8_t rxBuffer[RX_SERIAL_BUFFER_SIZE];
	uint8_t rxData; // Variable para recibir un byte
	uint16_t rxIndex = 0;

	UART_HandleTypeDef *huartSerial;

};

#endif /* UARTSERIAL_H_ */
