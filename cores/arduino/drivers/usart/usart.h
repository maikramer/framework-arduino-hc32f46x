/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once
#include <stdint.h>
#include "drivers/usart/usart.h"
#include "HardwareSerial.h"
#include "RingBuffer.h"
#include "usart_config.h"

class Usart : public HardwareSerial
{
public:
  /**
   * @brief construct a new Usart object
   * @param config pointer to the usart configuration struct
   * @param tx_pin gpio pin number for tx function
   * @param rx_pin gpio pin number for rx function
   */
  Usart(struct usart_config_t *config, uint16_t tx_pin, uint16_t rx_pin);
  void begin(uint32_t baud);
  void begin(uint32_t baud, uint16_t config);
  void begin(uint32_t baud, const stc_usart_uart_init_t *config);
  void end();
  int available();
  int availableForWrite();
  int peek();
  int read();
  void flush();
  size_t write(uint8_t ch);
  using Print::write; // pull in write(str) and write(buf, size) from Print
  operator bool() { return true; }

  bool connected() {};
  void flushTX() { flush(); };
  void msgDone() {};

  /**
   * @brief access the base usart config struct
   */
  const usart_config_t *c_dev(void) { return this->config; }

  /**
   * @brief get the last receive error
   * @note calling this function clears the error
   */
  const usart_receive_error_t getReceiveError(void);

private:
  // usart configuration struct
  usart_config_t *config;

  // tx / rx pin numbers
  uint16_t tx_pin;
  uint16_t rx_pin;

  // rx / tx buffers (unboxed from config)
  RingBuffer *rxBuffer;
  RingBuffer *txBuffer;

  // is initialized? (begin() called)
  bool initialized = false;
};

//
// global instances
//
#ifndef DISABLE_SERIAL_GLOBALS
extern HardwareSerial MSerial1;
extern HardwareSerial MSerial2;
extern HardwareSerial MSerial3;
extern HardwareSerial MSerial4;

#define Serial MSerial1
#endif

#ifdef __cplusplus
extern "C"
{
#endif

void uart1_init(void);
void Usart1RxIrqCallback(void);
void Usart1ErrIrqCallback(void);
void Usart1TxIrqCallback(void);
void Usart1TxCmpltIrqCallback(void);

void uart2_init(void);
void Usart2RxIrqCallback(void);
void Usart2ErrIrqCallback(void);
void Usart2TxIrqCallback(void);
void Usart2TxCmpltIrqCallback(void);

void uart3_init(void);
void Usart3RxIrqCallback(void);
void Usart3ErrIrqCallback(void);
void Usart3TxIrqCallback(void);
void Usart3TxCmpltIrqCallback(void);

void uart4_init(void);
void Usart4RxIrqCallback(void);
void Usart4ErrIrqCallback(void);
void Usart4TxIrqCallback(void);
void Usart4TxCmpltIrqCallback(void);

/*
 * Devices
 */
#define USART_RX_BUF_SIZE               128
#define USART_TX_BUF_SIZE               128

#ifdef __cplusplus
} // extern "C"
#endif
