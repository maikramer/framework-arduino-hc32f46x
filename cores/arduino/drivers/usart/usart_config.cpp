#include "usart.h"
#include "usart_config.h"
#include "../gpio/gpio.h"
#include "../../core_hooks.h"

// UART1
#define USART1_BAUDRATE                 (115200ul)
#define USART1_TX_PORT                  (PortA)  // Func_Grp1
#define USART1_TX_PIN                   (Pin09)
#define USART1_RX_PORT                  (PortA)  // Func_Grp1
#define USART1_RX_PIN                   (Pin15)
#define IRQ_INDEX_INT_USART1_RI         Int000_IRQn
#define IRQ_INDEX_INT_USART1_EI         Int001_IRQn
#define IRQ_INDEX_INT_USART1_TI         Int002_IRQn
#define IRQ_INDEX_INT_USART1_TCI        Int003_IRQn

// UART2
#define USART2_BAUDRATE                 (115200ul)
#define USART2_TX_PORT                  (PortA)  // Func_Grp1
#define USART2_TX_PIN                   (Pin02)
#define USART2_RX_PORT                  (PortA)  // Func_Grp1
#define USART2_RX_PIN                   (Pin03)
#define IRQ_INDEX_INT_USART2_RI         Int004_IRQn
#define IRQ_INDEX_INT_USART2_EI         Int005_IRQn
#define IRQ_INDEX_INT_USART2_TI         Int006_IRQn
#define IRQ_INDEX_INT_USART2_TCI        Int007_IRQn

// UART3
#define USART3_BAUDRATE                 (115200ul)
#define USART3_TX_PORT                  (PortA)  // Func_Grp1
#define USART3_TX_PIN                   (Pin11)
#define USART3_RX_PORT                  (PortA)  // Func_Grp1
#define USART3_RX_PIN                   (Pin12)
#define IRQ_INDEX_INT_USART3_RI         Int008_IRQn
#define IRQ_INDEX_INT_USART3_EI         Int009_IRQn
#define IRQ_INDEX_INT_USART3_TI         Int010_IRQn
#define IRQ_INDEX_INT_USART3_TCI        Int011_IRQn

// UART4
#define USART4_BAUDRATE                 (115200ul)
#define USART4_TX_PORT                  (PortB)  // Func_Grp2
#define USART4_TX_PIN                   (Pin10)
#define USART4_RX_PORT                  (PortH)  // Func_Grp2
#define USART4_RX_PIN                   (Pin02)
#define IRQ_INDEX_INT_USART4_RI         Int012_IRQn
#define IRQ_INDEX_INT_USART4_EI         Int013_IRQn
#define IRQ_INDEX_INT_USART4_TI         Int014_IRQn
#define IRQ_INDEX_INT_USART4_TCI        Int015_IRQn

#define IRQ_INDEX_INT_DMA2_TC0          Int016_IRQn
#define IRQ_INDEX_INT_DMA2_TC1          Int017_IRQn
#define IRQ_INDEX_INT_DMA2_TC2          Int018_IRQn

//
// USART configurations
//

usart_config_t USART1_config = {
    .peripheral = {
        .register_base = M4_USART1,
        .clock_id = PWC_FCG1_PERIPH_USART1,
        .tx_pin_function = Func_Usart1_Tx,
        .rx_pin_function = Func_Usart1_Rx
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_RI
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_EI
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_TI
        },
        .tx_complete = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART1_TCI
        },
    },
    .state = {
        .rx_buffer = new UsartBuffer(),
        .tx_buffer = new UsartBuffer(),
        .rx_error = usart_receive_error_t::None
    },
};

usart_config_t USART2_config = {
    .peripheral = {
        .register_base = M4_USART2,
        .clock_id = PWC_FCG1_PERIPH_USART2,
        .tx_pin_function = Func_Usart2_Tx,
        .rx_pin_function = Func_Usart2_Rx
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_08,
            .interrupt_source = INT_USART2_RI
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_EI
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_TI
        },
        .tx_complete = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART2_TCI
        },
    },
    .state = {
        .rx_buffer = new UsartBuffer(),
        .tx_buffer = new UsartBuffer(),
        .rx_error = usart_receive_error_t::None
    },
};

usart_config_t USART3_config = {
    .peripheral = {
        .register_base = M4_USART3,
        .clock_id = PWC_FCG1_PERIPH_USART3,
        .tx_pin_function = Func_Usart3_Tx,
        .rx_pin_function = Func_Usart3_Rx},
    .interrupts = {
        .rx_data_available = {.interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT, .interrupt_source = INT_USART3_RI},
        .rx_error = {.interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT, .interrupt_source = INT_USART3_EI},
        .tx_buffer_empty = {.interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT, .interrupt_source = INT_USART3_TI},
        .tx_complete = {.interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT, .interrupt_source = INT_USART3_TCI},
    },
    .state = {
        .rx_buffer = new UsartBuffer(),
        .tx_buffer = new UsartBuffer(),
        .rx_error = usart_receive_error_t::None,
    },
};

usart_config_t USART4_config = {
    .peripheral = {
        .register_base = M4_USART4,
        .clock_id = PWC_FCG1_PERIPH_USART4,
        .tx_pin_function = Func_Usart3_Tx,
        .rx_pin_function = Func_Usart3_Rx
    },
    .interrupts = {
        .rx_data_available = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_RI
        },
        .rx_error = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_EI
        },
        .tx_buffer_empty = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_TI
        },
        .tx_complete = {
            .interrupt_priority = DDL_IRQ_PRIORITY_DEFAULT,
            .interrupt_source = INT_USART4_TCI
        },
    },
    .state = {
        .rx_buffer = new UsartBuffer(),
        .tx_buffer = new UsartBuffer(),
        .rx_error = usart_receive_error_t::None,
    },
};
