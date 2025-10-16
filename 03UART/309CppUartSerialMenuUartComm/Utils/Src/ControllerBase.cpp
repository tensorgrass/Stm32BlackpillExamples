/*
 * ControllerBase.cpp
 *
 *  Created on: May 25, 2025
 *      Author: froilan
 */
#include <ControllerBase.hpp>
#include <cstring> // Para memcpy

ControllerBase::ControllerBase() {
  // TODO Auto-generated constructor stub
  option_list = {"A1", "A2"};
  tx_message = new StructMessage(); // Inicializar el mensaje de transmisión
  rx_message = new StructMessage(); // Inicializar el mensaje de recepción
  endTxTransaction(); // Asegurarse de que el mensaje de transmisión esté limpio
  step = 0; // Inicializa el paso
}

ControllerBase::~ControllerBase() {
  // TODO Auto-generated destructor stub
}


StructMessage *ControllerBase::getStruckTxMessage(){
  return tx_message;
}

void ControllerBase::setStruckTxMessage(StructMessage *message){
  tx_message = message;
}

StructMessage *ControllerBase::getStruckRxMessage(){
  return rx_message;
}

void ControllerBase::setStruckRxMessage(StructMessage *message){
  rx_message = message;
}

const std::vector<std::string>& ControllerBase::getOptionList() const {
  return option_list;
}

void ControllerBase::setOptionList(const std::vector<std::string>& opciones) {
  option_list = opciones;
}

bool ControllerBase::isOptionValid(const std::string& option) const {
  for (const auto& opt : option_list) {
    if (opt == option) {
      return true; // Opción válida
    }
  }
  return false; // Opción no válida
}

void ControllerBase::endTxTransaction() {
  setTxGroup('\0');
  setTxSubgroup('\0');
  setTxStep(0);
}


char ControllerBase::getTxGroup() const {
  return tx_message ? tx_message->group : '\0';
}

void ControllerBase::setTxGroup(char value) {
  if (tx_message) {
    tx_message->group = value;
  }
}

char ControllerBase::getTxSubgroup() const {
  return tx_message ? tx_message->subgroup : '\0';
}

void ControllerBase::setTxSubgroup(char value) {
  if (tx_message) {
    tx_message->subgroup = value;
  }
}

std::string ControllerBase::getTxMessage() const {
  if (tx_message) {
    return std::string(reinterpret_cast<const char*>(tx_message->message));
  }
  return "";
}

void ControllerBase::setTxMessage(const std::string& message) {
  if (tx_message) {
    // Suponiendo que StructMessage tiene un campo 'data' de tipo char[] y un tamaño fijo
    // Ajusta esto según la definición real de StructMessage
    size_t len = std::min(message.size(), sizeof(tx_message->message) - 1);
    memcpy(tx_message->message, message.c_str(), len);
    tx_message->message[len] = '\0';
  }
}

std::string ControllerBase::getTxError() const {
  if (tx_message) {
    return std::string(tx_message->error);
  }
  return "";
}

void ControllerBase::setTxError(const std::string& error) {
  if (tx_message) {
    size_t len = std::min(error.size(), sizeof(tx_message->error) - 1);
    memcpy(tx_message->error, error.c_str(), len);
    tx_message->error[len] = '\0';
  }
}

int ControllerBase::getTxStep() const {
  return step;
}

void ControllerBase::setTxStep(int value) {
  step = value;
}

// La serialización es simple: copiar los bytes del struct al buffer
size_t ControllerBase::serializeTxMessage(uint8_t* buffer, size_t buffer_size) {
  if (buffer_size < sizeof(StructMessage)) {
      // Buffer demasiado pequeño, manejar error
      return 0;
  }
  // Copiar la estructura byte a byte al buffer
  memcpy(buffer, tx_message, sizeof(StructMessage));
  return sizeof(StructMessage);
}

size_t ControllerBase::deserializeTxMessage(const uint8_t* buffer, size_t buffer_size) {
  if (buffer_size < sizeof(StructMessage)) {
    return 0;
  }
  memcpy(tx_message, buffer, sizeof(StructMessage));
  return sizeof(StructMessage);
}


void ControllerBase::endRxTransaction() {
  setRxGroup('\0');
  setRxSubgroup('\0');
  setRxStep(0);
}

char ControllerBase::getRxGroup() const {
  return rx_message ? rx_message->group : '\0';
}

void ControllerBase::setRxGroup(char value) {
  if (rx_message) {
    rx_message->group = value;
  }
}

char ControllerBase::getRxSubgroup() const {
  return rx_message ? rx_message->subgroup : '\0';
}

void ControllerBase::setRxSubgroup(char value) {
  if (rx_message) {
    rx_message->subgroup = value;
  }
}

std::string ControllerBase::getRxMessage() const {
  if (rx_message) {
    return std::string(reinterpret_cast<const char*>(rx_message->message));
  }
  return "";
}

void ControllerBase::setRxMessage(const std::string& message) {
  if (rx_message) {
    size_t len = std::min(message.size(), sizeof(rx_message->message) - 1);
    memcpy(rx_message->message, message.c_str(), len);
    rx_message->message[len] = '\0';
  }
}

std::string ControllerBase::getRxError() const {
  if (rx_message) {
    return std::string(rx_message->error);
  }
  return "";
}

void ControllerBase::setRxError(const std::string& error) {
  if (rx_message) {
    size_t len = std::min(error.size(), sizeof(rx_message->error) - 1);
    memcpy(rx_message->error, error.c_str(), len);
    rx_message->error[len] = '\0';
  }
}

int ControllerBase::getRxStep() const {
  return step;
}

void ControllerBase::setRxStep(int value) {
  step = value;
}

size_t ControllerBase::serializeRxMessage(uint8_t* buffer, size_t buffer_size) {
  if (buffer_size < sizeof(StructMessage)) {
    return 0;
  }
  memcpy(buffer, rx_message, sizeof(StructMessage));
  return sizeof(StructMessage);
}

size_t ControllerBase::deserializeRxMessage(const uint8_t* buffer, size_t buffer_size) {
  if (buffer_size < sizeof(StructMessage)) {
      // Buffer demasiado pequeño, manejar error
      return 0;
  }
  // Copiar el buffer byte a byte a la estructura
  memcpy(rx_message, buffer, sizeof(StructMessage));
  return sizeof(StructMessage);
}
