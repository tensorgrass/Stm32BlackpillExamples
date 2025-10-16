/*
 * UartComm.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#include <UartComm.hpp>


UartComm::UartComm() {
  // TODO Auto-generated constructor stub

}

UartComm::~UartComm() {
  // TODO Auto-generated destructor stub
}

void UartComm::init(UART_HandleTypeDef *huart, ControllerBase *controllerBase, UartSerial *uartSerialOriginal) {
  uartHandler = huart;
  controller = controllerBase;
  uartSerial = uartSerialOriginal; // Guardar el puntero al objeto UartSerial original

  HAL_UART_Receive_IT(uartHandler, rx_buffer, RX_BUFFER_SIZE);
}

void UartComm::sendData(){
  // Serializar el mensaje inicial para enviarlo
  controller->serializeTxMessage(tx_buffer, TX_BUFFER_SIZE);

  // Enviar el primer mensaje (bloqueante para asegurar que salga primero)
  HAL_UART_Transmit(uartHandler, tx_buffer, TX_BUFFER_SIZE, HAL_MAX_DELAY);
  HAL_Delay(100); // Pequeña espera para que el otro nodo reciba y procese
}

void UartComm::receiveData(UART_HandleTypeDef *huart){
  uartSerial->print("RecDat\r\n");
  // Deserializar el mensaje recibido
  controller->deserializeRxMessage(rx_buffer, RX_BUFFER_SIZE);
//  uartSerial->sendDataChar(controller->getRxGroup());
//  controller->setTxGroup(controller->getRxGroup());
//  uartSerial->sendDataChar(controller->getTxGroup());
//  controller->setTxSubgroup(controller->getRxSubgroup());
  // Reiniciar la recepción por interrupción para el siguiente mensaje
  // ¡Importante! Si no reinicias, solo recibirás un mensaje
  HAL_UART_Receive_IT(huart, rx_buffer, RX_BUFFER_SIZE);
}

