#pragma once

#include <hc32_ddl.h>
#include "RingBuffer.hpp"
#include "usart_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

// UART1
#define USART1_BAUDRATE (115200ul)
#define USART1_TX_PORT (PortA) // Func_Grp1
#define USART1_TX_PIN (Pin09)
#define USART1_RX_PORT (PortA) // Func_Grp1
#define USART1_RX_PIN (Pin15)
#define IRQ_INDEX_INT_USART1_RI Int000_IRQn
#define IRQ_INDEX_INT_USART1_EI Int001_IRQn
#define IRQ_INDEX_INT_USART1_TI Int002_IRQn
#define IRQ_INDEX_INT_USART1_TCI Int003_IRQn

// UART2
#define USART2_BAUDRATE (115200ul)
#define USART2_TX_PORT (PortA) // Func_Grp1
#define USART2_TX_PIN (Pin02)
#define USART2_RX_PORT (PortA) // Func_Grp1
#define USART2_RX_PIN (Pin03)
#define IRQ_INDEX_INT_USART2_RI Int004_IRQn
#define IRQ_INDEX_INT_USART2_EI Int005_IRQn
#define IRQ_INDEX_INT_USART2_TI Int006_IRQn
#define IRQ_INDEX_INT_USART2_TCI Int007_IRQn

// UART3
#define USART3_BAUDRATE (115200ul)
#define USART3_TX_PORT (PortA) // Func_Grp1
#define USART3_TX_PIN (Pin11)
#define USART3_RX_PORT (PortA) // Func_Grp1
#define USART3_RX_PIN (Pin12)
#define IRQ_INDEX_INT_USART3_RI Int008_IRQn
#define IRQ_INDEX_INT_USART3_EI Int009_IRQn
#define IRQ_INDEX_INT_USART3_TI Int010_IRQn
#define IRQ_INDEX_INT_USART3_TCI Int011_IRQn

// UART4
#define USART4_BAUDRATE (115200ul)
#define USART4_TX_PORT (PortB) // Func_Grp2
#define USART4_TX_PIN (Pin10)
#define USART4_RX_PORT (PortH) // Func_Grp2
#define USART4_RX_PIN (Pin02)
#define IRQ_INDEX_INT_USART4_RI Int012_IRQn
#define IRQ_INDEX_INT_USART4_EI Int013_IRQn
#define IRQ_INDEX_INT_USART4_TI Int014_IRQn
#define IRQ_INDEX_INT_USART4_TCI Int015_IRQn

#define IRQ_INDEX_INT_DMA2_TC0 Int016_IRQn
#define IRQ_INDEX_INT_DMA2_TC1 Int017_IRQn
#define IRQ_INDEX_INT_DMA2_TC2 Int018_IRQn

// usart device variables
#define USART_DEV_VARS(nr)           \
    extern usart_config_t usart##nr; \
    extern struct usart_config_t *USART##nr;

    USART_DEV_VARS(1)
    USART_DEV_VARS(2)
    USART_DEV_VARS(3)

    // public api
    void usart_init(struct usart_config_t *config);

    void usart_set_baud_rate(struct usart_config_t *config, uint32_t baud);

    void usart_enable(struct usart_config_t *config);

    void usart_disable(struct usart_config_t *config);

    uint32_t usart_tx(struct usart_config_t *config, const uint8_t *buf, uint32_t len);

    uint32_t usart_rx(struct usart_config_t *config, uint8_t *buf, uint32_t len);

    void usart_putudec(struct usart_config_t *config, uint32_t val);

size_t usart_print(struct usart_config_t *config, const char *msg);

size_t usart_printf(struct usart_config_t *config, const char *fmt, ...);

    /**
     * @brief Disable all serial ports.
     */
    static inline void usart_disable_all(void)
    {
        usart_disable(&USART1_config);
        usart_disable(&USART2_config);
        usart_disable(&USART4_config);
    }

    /**
     * @brief Transmit one character on a serial port.
     *
     * This function blocks until the character has been queued
     * for transmission.
     *
     * @param config Serial port to send on.
     * @param byte Byte to transmit.
     */
    static inline void usart_putc(struct usart_config_t *config, uint8_t byte)
    {
        usart_tx(config, &byte, 1);
    }

    /**
     * @brief Transmit a character string on a serial port.
     *
     * This function blocks until str is completely transmitted.
     *
     * @param config Serial port to send on
     * @param str String to send
     */
    static inline void usart_putstr(struct usart_config_t *config, const char *str)
    {
        uint32_t i = 0;
        while (str[i] != '\0')
        {
            usart_putc(config, str[i++]);
        }
    }

    /**
     * @brief Read one character from a serial port.
     *
     * It's not safe to call this function if the serial port has no data
     * available.
     *
     * @param config Serial port to read from
     * @return byte read
     * @see usart_data_available()
     */
    static inline uint8_t usart_getc(struct usart_config_t *config)
    {
        uint8_t ch;
        config->state.rx_buffer->pop(ch);
        return ch;
    }

    /*
     * Roger Clark. 20141125,
     * added peek function.
     * @param config Serial port to read from
     * @return byte read
     */
    static inline int usart_peek(struct usart_config_t *config)
    {
        return (int)(*config->state.rx_buffer->peek());
    }

    /**
     * @brief Return the amount of data available in a serial port's RX buffer.
     * @param config Serial port to check
     * @return Number of bytes in config's RX buffer.
     */
    static inline uint32_t usart_data_available(struct usart_config_t *config)
    {
        return config->state.rx_buffer->count();
    }

    /**
     * @brief Discard the contents of a serial port's RX buffer.
     * @param config Serial port whose buffer to empty.
     */
    static inline void usart_reset_rx(struct usart_config_t *config)
    {
        config->state.rx_buffer->clear();
    }

    /**
     * @brief Discard the contents of a serial port's RX buffer.
     * @param config Serial port whose buffer to empty.
     */
    static inline void usart_reset_tx(struct usart_config_t *config)
    {
        config->state.tx_buffer->clear();
    }

    /**
     * USART transmit hook
     *
     * @param ch the data byte to be transmitted
     * @param usart the usart channel. ([1,2,3,4]; 1 = DWIN, 2 = PRINT)
     */
    __attribute__((weak)) extern void usart_tx_irq_hook(uint8_t ch, uint8_t usart);

    /**
     * USART receive hook
     *
     * @param ch the data byte that was received
     * @param usart the usart channel. ([1,2,3,4]; 1 = DWIN, 2 = PRINT)
     */
    __attribute__((weak)) extern void usart_rx_irq_hook(uint8_t ch, uint8_t usart);

    /**
     * map usart device registers to usart channel number
     */
    static inline uint8_t usart_dev_to_channel(M4_USART_TypeDef *dev_regs)
    {
        if (dev_regs == M4_USART1)
            return 1;
        if (dev_regs == M4_USART2)
            return 2;
        if (dev_regs == M4_USART3)
            return 3;
        if (dev_regs == M4_USART4)
            return 4;

        return 0xff;
    }

    static inline void usart_tx_irq(struct usart_config_t *config)
    {
        uint8_t ch;
        if (config->state.tx_buffer->pop(ch))
        {
            usart_tx_irq_hook(ch, usart_dev_to_channel(config->peripheral.register_base));
            USART_SendData(config->peripheral.register_base, ch);
        }
        else
        {
            USART_FuncCmd(config->peripheral.register_base, UsartTxEmptyInt, Disable);
            USART_FuncCmd(config->peripheral.register_base, UsartTxCmpltInt, Enable);
        }
    }

    static inline void usart_rx_irq(struct usart_config_t *config)
    {
        uint8_t ch = (uint8_t)USART_RecData(config->peripheral.register_base);
        usart_rx_irq_hook(ch, usart_dev_to_channel(config->peripheral.register_base));
        config->state.rx_buffer->push(ch);
    }

#ifdef __cplusplus
} // extern "C"
#endif
