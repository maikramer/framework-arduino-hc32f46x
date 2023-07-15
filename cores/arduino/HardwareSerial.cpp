#include "HardwareSerial.h"
#include "drivers/usart/usart.h"
#include "drivers/gpio/gpio.h"
#include <stdarg.h>

//
// global instances
//
#ifndef DISABLE_SERIAL_GLOBALS
HardwareSerial Serial1(&USART1_config, VARIANT_USART1_TX_PIN, VARIANT_USART1_RX_PIN);
HardwareSerial Serial2(&USART2_config, VARIANT_USART2_TX_PIN, VARIANT_USART2_RX_PIN);
HardwareSerial Serial3(&USART3_config, VARIANT_USART3_TX_PIN, VARIANT_USART3_RX_PIN);
HardwareSerial Serial4(&USART4_config, VARIANT_USART4_TX_PIN, VARIANT_USART4_RX_PIN);
#endif

HardwareSerial::HardwareSerial(struct usart_config_t *config,
                               uint8_t tx_pin,
                               uint8_t rx_pin) {
    this->config = config;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
}

void HardwareSerial::init(struct usart_config_t *config,
                          uint8_t tx_pin,
                          uint8_t rx_pin) {
    this->config = config;
    // unpack rx and tx buffers from usart config

    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
}

void HardwareSerial::begin(uint32_t baud) {
    begin(baud, 0);
}

void HardwareSerial::begin(uint32_t baud, uint8_t config) {
    // init IO pins
    // set IO pin functions
    GPIO_SetFunc(this->tx_pin, this->config->peripheral.tx_pin_function, Disable);
    GPIO_SetFunc(this->rx_pin, this->config->peripheral.rx_pin_function, Disable);
    usart_init(this->config);
    usart_set_baud_rate(this->config, baud);
    usart_enable(this->config);
}

void HardwareSerial::end(void) {
    usart_disable(this->config);
}

int HardwareSerial::read(void) {
    if (usart_data_available(config) > 0) {
        return usart_getc(config);
    } else {
        return -1;
    }
}

int HardwareSerial::available(void) {
    return usart_data_available(this->config);
}

int HardwareSerial::peek(void) {
    return usart_peek(this->config);
}

int HardwareSerial::availableForWrite(void) {
    return this->config->state.tx_buffer->capacity() - this->config->state.tx_buffer->count();
}

size_t HardwareSerial::write(unsigned char ch) {
    usart_putc(this->config, ch);
    return 1;
}

void HardwareSerial::flush(void) {
    while (!this->config->state.tx_buffer->isEmpty());
}

size_t HardwareSerial::printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(printf_buffer, HSERIAL_PRINTF_BUFFER_SIZE, fmt, args);
    va_end(args);

    return usart_print(this->config, printf_buffer);
}