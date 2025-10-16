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

void main_cpp(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2) {
  std::string menu = "\r\n"
                     "Master\r\n"
                     "[A1] Prueba de conexión     [A2] Prueba de conexion multiple\r\n"
                     "Elegir una opcion:\r\n";
  uartSerial.init(huart1, &controller, menu);
  uartComm.init(huart2, &controller, &uartSerial);

  while( 1 ) {
    if (controller.getTxGroup() == 'A' && controller.getTxSubgroup() == '1') {
      if (controller.getTxStep() == 0) {
        controller.setTxStep(1); // Cambia el paso para evitar reentradas
        uartSerial.print("Master: Step 0\r\n");
        HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);

        // Inicializar el primer mensaje a enviar
        controller.setTxMessage("Hola");
        controller.setTxError("OK");

        uartComm.sendData();

      }
      else if (controller.getTxStep() == 2) {

        message_counter++;

        // Imprimir el mensaje recibido por SWV o Debug (si tienes configurado)
        // O toggling de LED para indicar recepción
        HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);


        uartSerial.print("Master: Step 2\r\n");
        controller.setTxStep(3);
        controller.endTxTransaction(); // Finaliza la transacción
      }
      else {
        // Si no hay mensaje, puedes hacer otras cosas o simplemente esperar
        HAL_Delay(10); // Pequeña espera para no saturar el bucle
      }
    }
    else {
      // Si no hay mensaje, puedes hacer otras cosas o simplemente esperar
      HAL_Delay(10); // Pequeña espera para no saturar el bucle
    }
    //		for (int i = 0; i < 2; i++) {
    //			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //			HAL_Delay(200);
    //			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //			HAL_Delay(200);
    //		}
    //		HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //		HAL_Delay(500);
    //		HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    //		HAL_Delay(200);
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
