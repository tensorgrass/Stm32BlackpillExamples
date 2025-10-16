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

void UartSerial::init(UART_HandleTypeDef *huart) {
  menu_string = "\r\n"
                "[A1] Prueba de conexión     [A2] Prueba de conexion multiple\r\n"
                "Elegir una opcion:\r\n";
  lista_opciones = {"A1", "A2"};
	huartSerial = huart;
	HAL_UART_Receive_IT(huart, &rxData, 1);
}

void UartSerial::receiveData(UART_HandleTypeDef *huart){
    if (rxData == '\n' || rxData == '\r') // Verifica si llegó el carácter de cambio de línea
    {
      rxBuffer[rxIndex] = '\0'; // Termina la cadena
      // Procesa el mensaje recibido en rxBuffer
      rxIndex = 0; // Reinicia el índice para el próximo mensaje
      std::string selected_option(reinterpret_cast<const char*>(rxBuffer));
      bool opcion_encontrada = false;
      for (const std::string& opcion : lista_opciones) { // 'const std::string&' para eficiencia y seguridad
        if (opcion == selected_option) {
          opcion_encontrada = true;
          break;
        }
      }
      if (opcion_encontrada) {
        std::string texto_opcion_seleccionada = "\r\nOpción seleccionada " + selected_option + "\r\n";
//        HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(texto_opcion_seleccionada.data()), texto_opcion_seleccionada.size());
        sendDataString(huart, texto_opcion_seleccionada);
      }
      else {
//        HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(menu_string.data()), menu_string.size());
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

void UartSerial::sendDataString(UART_HandleTypeDef *huart, std::string string_to_send) {
  HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(string_to_send.data()), string_to_send.size());
}

void UartSerial::sendDataChar(UART_HandleTypeDef *huart, unsigned char char_to_send ){
  HAL_UART_Transmit_IT(huart, &char_to_send, 1);
}
