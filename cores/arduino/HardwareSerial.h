#pragma once
#include "Print.h"
#include "Stream.h"
#include "RingBuffer.hpp"

#define HSERIAL_PRINTF_BUFFER_SIZE 256

typedef uint16_t tx_buffer_index_t;
typedef uint16_t rx_buffer_index_t;

#define DEFINE_HWSERIAL(name, n)                 \
    HardwareSerial name(USART##n,                \
                        BOARD_USART##n##_TX_PIN, \
                        BOARD_USART##n##_RX_PIN)

class HardwareSerial : public Stream
{

public:
    HardwareSerial(struct usart_config_t *config,
                   uint8_t tx_pin,
                   uint8_t rx_pin);

    void init(struct usart_config_t *config,
              uint8_t tx_pin,
              uint8_t rx_pin);
    void begin(uint32_t baud);
    void begin(uint32_t baud, uint8_t config);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    int availableForWrite(void);
    virtual void flush(void);
    size_t write(unsigned char ch);

    int txPin(void) { return this->tx_pin; }
    int rxPin(void) { return this->rx_pin; }
    size_t printf(const char *fmt, ...);

    operator bool()
    {
        return true;
    }

    // Escape hatch into libmaple
    struct usart_config_t *c_dev(void) { return this->config; }

  private:
    char printf_buffer[HSERIAL_PRINTF_BUFFER_SIZE];
    struct usart_config_t *config;
    uint8_t tx_pin;
    uint8_t rx_pin;
};

//
// global instances
//
#ifndef DISABLE_SERIAL_GLOBALS
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;

#define Serial Serial1
#endif
