/*
 * UartBase.hpp
 *
 *  Created on: May 25, 2025
 *      Author: froilan
 */
#ifndef INC_UARTBASE_HPP_
#define INC_UARTBASE_HPP_

#include <string>
#include <stm32f4xx_hal.h>
#include <ControllerBase.hpp>

class UartBase {

public:
  UartBase();
  virtual ~UartBase();

  virtual UART_HandleTypeDef *getUartHandler();
  virtual void setUartHandler(UART_HandleTypeDef *huart);

  virtual ControllerBase *getController();
  virtual void setController(ControllerBase *controllerBase);

  virtual void sendDataString(std::string string_to_send);
  virtual void sendDataString(UART_HandleTypeDef *huart, std::string string_to_send);
  virtual void sendDataChar(unsigned char char_to_send );
  virtual void sendDataChar(UART_HandleTypeDef *huart, unsigned char char_to_send );

protected:
  UART_HandleTypeDef *uartHandler;
  ControllerBase *controller;

};

#endif /* INC_UARTBASE_HPP_ */
