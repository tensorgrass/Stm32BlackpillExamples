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

class UartSerial {
public:
	UartSerial();
	virtual ~UartSerial();
	virtual void init(UART_HandleTypeDef *huart);
	virtual void receiveData(UART_HandleTypeDef *huart);

private:
	#define RX_SERIAL_BUFFER_SIZE 100
	unsigned char rxBuffer[RX_SERIAL_BUFFER_SIZE];
	unsigned char rxData; // Variable para recibir un byte
	uint16_t rxIndex = 0;

	UART_HandleTypeDef *huartSerial;
	std::string menu_string;

	std::vector<std::string> lista_opciones;

  virtual void sendDataString(UART_HandleTypeDef *huart, std::string string_to_send);
  virtual void sendDataChar(UART_HandleTypeDef *huart, unsigned char char_to_send );
};

#endif /* UARTSERIAL_H_ */
