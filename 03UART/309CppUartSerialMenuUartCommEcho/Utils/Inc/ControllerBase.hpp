/*
 * ControllerBase.hpp
 *
 *  Created on: May 25, 2025
 *      Author: froilan
 */

#ifndef INC_CONTROLLERBASE_HPP_
#define INC_CONTROLLERBASE_HPP_

#include <string>
#include <message_types.hpp>
#include <vector>


class ControllerBase {
public:
  ControllerBase();
  virtual ~ControllerBase();

  virtual StructMessage *getStruckTxMessage();
  virtual void setStruckTxMessage(StructMessage *message);

  virtual StructMessage *getStruckRxMessage();
  virtual void setStruckRxMessage(StructMessage *message);

  virtual const std::vector<std::string>& getOptionList() const;
  virtual void setOptionList(const std::vector<std::string>& options);
  virtual bool isOptionValid(const std::string& option) const;

  // Método para indicar el final de la transacción
  virtual void endTxTransaction();

  virtual char getTxGroup() const;
  virtual void setTxGroup(char value);

  virtual char getTxSubgroup() const;
  virtual void setTxSubgroup(char value);

  virtual std::string getTxMessage() const;
  virtual void setTxMessage(const std::string& message);

  virtual std::string getTxError() const;
  virtual void setTxError(const std::string& message);

  virtual int getTxStep() const;
  virtual void setTxStep(int value);

  virtual size_t serializeTxMessage(uint8_t* buffer, size_t buffer_size);
  virtual size_t deserializeTxMessage(const uint8_t* buffer, size_t buffer_size);


  // Métodos equivalentes para Rx
  virtual void endRxTransaction();

  virtual char getRxGroup() const;
  virtual void setRxGroup(char value);

  virtual char getRxSubgroup() const;
  virtual void setRxSubgroup(char value);

  virtual std::string getRxMessage() const;
  virtual void setRxMessage(const std::string& message);

  virtual std::string getRxError() const;
  virtual void setRxError(const std::string& message);

  virtual int getRxStep() const;
  virtual void setRxStep(int value);

  virtual size_t serializeRxMessage(uint8_t* buffer, size_t buffer_size);
  virtual size_t deserializeRxMessage(const uint8_t* buffer, size_t buffer_size);

protected:
  StructMessage *tx_message;          // Mensaje a transmitir
  StructMessage *rx_message;          // Mensaje recibido

  int step;
	std::vector<std::string> option_list;

  int id_counter = 0; // Contador de IDs para mensajes

private:


};



#endif /* INC_CONTROLLERBASE_HPP_ */
