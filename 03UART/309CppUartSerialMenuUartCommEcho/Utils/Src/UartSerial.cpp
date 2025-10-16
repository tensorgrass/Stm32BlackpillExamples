/*
 * UartSerial.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#include <UartSerial.hpp>


UartSerial::UartSerial() {
	// TODO Auto-generated constructor stub

}

UartSerial::~UartSerial() {
	// TODO Auto-generated destructor stub
}

void UartSerial::init(UART_HandleTypeDef *huart, ControllerBase *controllerBase, std::string menu) {
  uartHandler = huart;
  controller = controllerBase;
  menu_string = menu;
  
	HAL_UART_Receive_IT(uartHandler, &rxData, 1);
}

void UartSerial::receiveData(UART_HandleTypeDef *huart){
    if (rxData == '\n' || rxData == '\r') // Verifica si llegó el carácter de cambio de línea
    {
      rxBuffer[rxIndex] = '\0'; // Termina la cadena
      // Procesa el mensaje recibido en rxBuffer
      rxIndex = 0; // Reinicia el índice para el próximo mensaje
      std::string selected_option(reinterpret_cast<const char*>(rxBuffer));
      if (!selected_option.empty()) {
        selected_option[0] = std::toupper(static_cast<unsigned char>(selected_option[0]));
      }

      if (controller->isOptionValid(selected_option)) {
        // opcionSeleccionada = selected_option;
        controller->setTxGroup(selected_option[0]); // Asigna el primer carácter como grupo
        controller->setTxSubgroup(selected_option[1]); // Asigna el segundo carácter como subgrupo
        controller->setTxStep(0); // Reinicia el paso a 0

//        std::string texto_opcion_seleccionada = "\r\nSelected option " + selected_option + "\r\n";
//        sendDataString(huart, texto_opcion_seleccionada);
        std::string texto_opcion_seleccionada = "\r\n";
        sendDataString(huart, texto_opcion_seleccionada);
      }
      else {
        sendDataString(huart, menu_string);
      }
    }
    else if (rxData == '\177' || rxData == '\b') // Verifica si llegó el carácter de cambio de línea
    {
    	if (rxIndex > 0) {
    	  rxIndex--;
    	    std::string crlf_sequence = "\b \b";
//    	    HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(crlf_sequence.data()), crlf_sequence.size());
    	    sendDataString(huart, crlf_sequence);
    	}
    }
    else
    {
      if (rxIndex < RX_SERIAL_BUFFER_SIZE - 1) // Evita desbordamiento del búfer
      {
        rxBuffer[rxIndex++] = rxData; // Almacena el byte recibido
      }
      HAL_UART_Transmit_IT(huart, &rxData, 1);
      sendDataChar(huart, rxData);
    }
    // Reinicia la recepción de un nuevo byte
    HAL_UART_Receive_IT(huart, &rxData, 1);
}

void UartSerial::print(std::string string_to_send) {
  HAL_UART_Transmit(uartHandler, reinterpret_cast<const unsigned char*>(string_to_send.data()), string_to_send.size(), 1);
}
