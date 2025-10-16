/*
 * maincpp.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#include <maincpp.hpp>
#include <message_types.hpp>
#include <ControllerBase.hpp>
#include <UartSerial.hpp>
#include <UartComm.hpp>
#include <main.h>


UartSerial uartSerial;
UartComm uartComm;
ControllerBase controller;

// Variable para contar los mensajes enviados/recibidos
volatile int message_counter = 0;

void main_cpp(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2)
{
  std::string menu = "\r\n"
                     "Slave ECO\r\n";
  uartSerial.init(huart1, &controller, menu);
  uartComm.init(huart2, &controller, &uartSerial);

  while( 1 ) {
    if (controller.getRxGroup() == 'A' && controller.getRxSubgroup() == '1') {
      if (controller.getRxStep() == 2) {

        controller.setTxStep(3); // Cambia el paso para evitar reentradas
        uartSerial.print("Slave Eco: Step 0\r\n");
        HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);

        controller.setTxGroup(controller.getRxGroup());
        controller.setTxSubgroup(controller.getRxSubgroup());
        controller.setTxMessage(controller.getRxMessage() + " ECO");
        controller.setTxError("OK");

        uartComm.sendData();
        controller.endTxTransaction(); // Finaliza la transacción de envío
        controller.endRxTransaction(); // Finaliza la transacción de envío
      } else {
        // Si no hay mensaje, puedes hacer otras cosas o simplemente esperar
        HAL_Delay(1); // Pequeña espera para no saturar el bucle
      }
    } else {
      // Si no hay mensaje, puedes hacer otras cosas o simplemente esperar
      HAL_Delay(1); // Pequeña espera para no saturar el bucle
    }
    //    for (int i = 0; i < 2; i++) {
    //      HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //      HAL_Delay(200);
    //      HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //      HAL_Delay(200);
    //    }
    //    HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //    HAL_Delay(500);
    //    HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //    HAL_Delay(200);
  }
}

void HAL_UART_RxCpltCallback_cpp(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) // Verifica que sea el UART correcto
  {
    // Llama al método de la clase UartSerial para manejar la recepción
    uartSerial.receiveData(huart);
  }
  else if (huart->Instance == USART2) {
    // Hemos recibido un mensaje completo
    controller.setTxStep(2); // Cambia el paso para indicar que se ha recibido un mensaje
    uartComm.receiveData(huart);
  }
}
