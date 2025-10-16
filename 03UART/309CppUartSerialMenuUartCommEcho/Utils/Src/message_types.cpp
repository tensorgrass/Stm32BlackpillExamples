/*
 * message_types.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */
#include "message_types.hpp"
#include <cstring> // Para memcpy

// La serialización es simple: copiar los bytes del struct al buffer
size_t serialize_message(const StructMessage& msg, uint8_t* buffer, size_t buffer_size) {
    if (buffer_size < sizeof(StructMessage)) {
        // Buffer demasiado pequeño, manejar error
        return 0;
    }
    // Copiar la estructura byte a byte al buffer
    memcpy(buffer, &msg, sizeof(StructMessage));
    return sizeof(StructMessage);
}

// La deserialización es simple: copiar los bytes del buffer al struct
size_t deserialize_message(const uint8_t* buffer, StructMessage& msg, size_t buffer_size) {
    if (buffer_size < sizeof(StructMessage)) {
        // Buffer demasiado pequeño, manejar error
        return 0;
    }
    // Copiar el buffer byte a byte a la estructura
    memcpy(&msg, buffer, sizeof(StructMessage));
    return sizeof(StructMessage);
}



