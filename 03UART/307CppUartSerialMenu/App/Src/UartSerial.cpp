/*
 * UartSerial.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#include <UartSerial.hpp>
//#include "string.h" // Needed for strlen if you use it for C
//#include "cstring" // Needed for strlen if you use it for C++
//#include <string>    // Required for std::string and strlen
//#include <iostream>  // For printing (optional)
//#include <cstdint>   // Required for uint8_t

UartSerial::UartSerial() {
	// TODO Auto-generated constructor stub

}

UartSerial::~UartSerial() {
	// TODO Auto-generated destructor stub
}

void UartSerial::init(UART_HandleTypeDef *huart) {
//  menu = (const unsigned char*)"\r\n";
//  menu = (const unsigned char*)"\r\n[A1] Prueba de conexión     [A2] Prueba de conexion multiple\r\nElegir una opcion:\r\n";
//  menu_string = std::string("\r\n") +
//                std::string("[A1] Prueba de conexión     [A2] Prueba de conexion multiple\r\n") +
//                std::string("Elegir una opcion:\r\n");
  menu_string = "\r\n"
                "[A1] Prueba de conexión     [A2] Prueba de conexion multiple\r\n"
                "Elegir una opcion:\r\n";
	huartSerial = huart;
	HAL_UART_Receive_IT(huart, &rxData, 1);
}

void UartSerial::receiveData(UART_HandleTypeDef *huart){
    if (rxData == '\n' || rxData == '\r') // Verifica si llegó el carácter de cambio de línea
    {
      rxBuffer[rxIndex] = '\0'; // Termina la cadena
      // Procesa el mensaje recibido en rxBuffer
      rxIndex = 0; // Reinicia el índice para el próximo mensaje
//      const unsigned char crlf_sequence[] = "\r\n";
//      HAL_UART_Transmit_IT(huart, crlf_sequence, strlen((const char*)crlf_sequence));
//      HAL_UART_Transmit_IT(huart, menu, strlen((const char*)menu));
      HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(menu_string.data()), menu_string.size());
    }
    else if (rxData == '\177' || rxData == '\b') // Verifica si llegó el carácter de cambio de línea
    {
    	if (rxIndex > 0) {
    	  rxIndex--;
//        const unsigned char crlf_sequence[] = "\b \b";
//        HAL_UART_Transmit_IT(huart, crlf_sequence, strlen((const char*)crlf_sequence));
    	    std::string crlf_sequence = "\b \b";
    	    HAL_UART_Transmit_IT(huart, reinterpret_cast<const unsigned char*>(crlf_sequence.data()), crlf_sequence.size());
    	}
    }
    else
    {
      if (rxIndex < RX_SERIAL_BUFFER_SIZE - 1) // Evita desbordamiento del búfer
      {
        rxBuffer[rxIndex++] = rxData; // Almacena el byte recibido
      }
      HAL_UART_Transmit_IT(huart, &rxData, 1);
    }
    // Reinicia la recepción de un nuevo byte
    HAL_UART_Receive_IT(huart, &rxData, 1);
}
