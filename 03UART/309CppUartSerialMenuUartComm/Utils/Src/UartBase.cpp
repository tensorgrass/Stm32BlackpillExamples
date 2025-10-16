/*
 * UartBase.cpp
 *
 *  Created on: May 25, 2025
 *      Author: froilan
 */
#include <UartBase.hpp>

UartBase::UartBase() {
  // TODO Auto-generated constructor stub
}

UartBase::~UartBase() {
  // TODO Auto-generated destructor stub
}


UART_HandleTypeDef *UartBase::getUartHandler(){
  return uartHandler;
}

void UartBase::setUartHandler(UART_HandleTypeDef *huart){
  uartHandler = huart;
}

ControllerBase *UartBase::getController(){
  return controller;
}

void UartBase::setController(ControllerBase *controllerBase){
  controller = controllerBase;
}

void UartBase::sendDataString(std::string string_to_send) {
  sendDataString(uartHandler, string_to_send);
}

void UartBase::sendDataString(UART_HandleTypeDef *huart, std::string string_to_send) {
  HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(string_to_send.data()), string_to_send.size());
}

void UartBase::sendDataChar(unsigned char char_to_send ){
  sendDataChar(uartHandler, char_to_send);
}

void UartBase::sendDataChar(UART_HandleTypeDef *huart, unsigned char char_to_send ){
  HAL_UART_Transmit_IT(huart, &char_to_send, 1);
}

