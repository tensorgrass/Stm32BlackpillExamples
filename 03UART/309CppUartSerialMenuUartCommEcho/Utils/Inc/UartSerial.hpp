/*
 * UartSerial.h
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#ifndef UARTSERIAL_H_
#define UARTSERIAL_H_

#include <stm32f4xx_hal.h>
#include <string>
#include <cstdint>   // Required for uint8_t
#include <vector>
#include <UartBase.hpp>

class UartSerial : public UartBase {
public:
	UartSerial();
	virtual ~UartSerial();
	virtual void init(UART_HandleTypeDef *huart, ControllerBase *controllerBase, std::string menu);
	virtual void receiveData(UART_HandleTypeDef *huart);
	virtual void print(std::string string_to_send);


private:
	#define RX_SERIAL_BUFFER_SIZE 100
	unsigned char rxBuffer[RX_SERIAL_BUFFER_SIZE];
	unsigned char rxData; // Variable para recibir un byte
	uint16_t rxIndex = 0;

	std::string menu_string;



};

#endif /* UARTSERIAL_H_ */
