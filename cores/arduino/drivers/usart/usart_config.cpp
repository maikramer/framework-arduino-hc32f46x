#include "usart.h"
#include "usart_config.h"
#include "../gpio/gpio.h"
#include "../../core_hooks.h"

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
