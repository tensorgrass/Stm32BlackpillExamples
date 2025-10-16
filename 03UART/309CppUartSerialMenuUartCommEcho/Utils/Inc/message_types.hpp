/*
 * message_types.hpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#ifndef INC_MESSAGE_TYPES_HPP_
#define INC_MESSAGE_TYPES_HPP_

#include <cstdint> // Para uint8_t
#include <string>  // Para std::string

// Definición del struct. Importante: no usar std::string directamente aquí
// en un struct que será enviado byte a byte, ya que std::string
// es un objeto complejo con punteros. Usamos arrays de char fijos.
struct StructMessage {
    int id;               // 4 bytes
    char group;           // 1 byte
    char subgroup;        // 1 byte
    char message[20];     // 20 bytes
    char error[20];       // 20 bytes
    // Total: 4 + 1 + 1 + 20 + 20 = 46 bytes

    // Constructor por defecto
    StructMessage() : id(0), group(0), subgroup(0) {
        // Inicializar arrays de char a '\0'
        for (int i = 0; i < 20; ++i) {
            message[i] = '\0';
            error[i] = '\0';
        }
    }
};

// Funciones para serializar/deserializar
// Devuelve el número de bytes serializados (sizeof(StructMensaje))
size_t serialize_message(const StructMessage& msg, uint8_t* buffer, size_t buffer_size);

// Devuelve el número de bytes deserializados (sizeof(StructMensaje))
size_t deserialize_message(const uint8_t* buffer, StructMessage& msg, size_t buffer_size);





#endif /* INC_MESSAGE_TYPES_HPP_ */
