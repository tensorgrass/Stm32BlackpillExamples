/*
 * UartComm.h
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#ifndef UARTCOMM_H_
#define UARTCOMM_H_

#include <stm32f4xx_hal.h>
#include <string>
#include <cstdint>   // Required for uint8_t
#include <vector>
#include <UartBase.hpp>
#include <UartSerial.hpp>

#define RX_BUFFER_SIZE  sizeof(StructMessage) // El tamaño del buffer de recepción es el tamaño del struct
#define TX_BUFFER_SIZE  sizeof(StructMessage) // El tamaño del buffer de transmisión es el tamaño del struct

class UartComm : public UartBase {
public:
  UartComm();
  virtual ~UartComm();
  virtual void init(UART_HandleTypeDef *huart, ControllerBase *controllerBase, UartSerial *uartSerialOriginal);
  virtual void receiveData(UART_HandleTypeDef *huart);
  virtual void sendData();

private:
  UartSerial *uartSerial; // Puntero al objeto UartSerial original

  uint8_t rx_buffer[RX_BUFFER_SIZE]; // Buffer para la recepción UART
  uint8_t tx_buffer[TX_BUFFER_SIZE]; // Buffer para la transmisión UART



};

#endif /* UARTCOMM_H_ */
