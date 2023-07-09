#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

#define PRINTF_BUFFER_SIZE 256

//
// USART1_config callbacks
//
void Usart1RxIrqCallback(void)
{
	usart_rx_irq(&USART1_config);
}

void Usart1ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART1, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART1, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART1, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART1, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART1, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART1, UsartOverrunErr);
	}
}

void Usart1TxIrqCallback(void)
{
	usart_tx_irq(&USART1_config);
}

void Usart1TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART1, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART1, UsartTx, Disable);
}

//
// USART2_config callbacks
//
void Usart2RxIrqCallback(void)
{
	usart_rx_irq(&USART2_config);
}

void Usart2ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART2, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART2, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART2, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART2, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART2, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART2, UsartOverrunErr);
	}
}

void Usart2TxIrqCallback(void)
{
	usart_tx_irq(&USART2_config);
}

void Usart2TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART2, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART2, UsartTx, Disable);
}

//
// USART3_config callbacks
//
void Usart3RxIrqCallback(void)
{
	usart_rx_irq(&USART3_config);
}

void Usart3ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART3, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART3, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART3, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART3, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART3, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART3, UsartOverrunErr);
	}
}

void Usart3TxIrqCallback(void)
{
	usart_tx_irq(&USART3_config);
}

void Usart3TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART3, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART3, UsartTx, Disable);
}

//
// USART4_config callbacks
//
void Usart4RxIrqCallback(void)
{
	usart_rx_irq(&USART4_config);
}

void Usart4ErrIrqCallback(void)
{
	if (USART_GetStatus(M4_USART4, UsartFrameErr) == Set)
	{
		USART_ClearStatus(M4_USART4, UsartFrameErr);
	}

	if (USART_GetStatus(M4_USART4, UsartParityErr) == Set)
	{
		USART_ClearStatus(M4_USART4, UsartParityErr);
	}

	if (USART_GetStatus(M4_USART4, UsartOverrunErr) == Set)
	{
		USART_ClearStatus(M4_USART4, UsartOverrunErr);
	}
}

void Usart4TxIrqCallback(void)
{
	usart_tx_irq(&USART4_config);
}

void Usart4TxCmpltIrqCallback(void)
{
	USART_FuncCmd(M4_USART4, UsartTxCmpltInt, Disable);
	USART_FuncCmd(M4_USART4, UsartTx, Disable);
}

inline void usart_irq_resign(usart_interrupt_config_t &irq, const char *name)
{
	// disable interrupt and clear pending
	NVIC_DisableIRQ(irq.interrupt_number);
	NVIC_ClearPendingIRQ(irq.interrupt_number);
	enIrqResign(irq.interrupt_number);
}

//
// public api
//
void usart_init(struct usart_config_t *config)
{
	const stc_usart_uart_init_t stcInitCfg = {
		.enClkMode = UsartIntClkCkNoOutput,
		.enClkDiv = UsartClkDiv_16,
		.enDataLength = UsartDataBits8,
		.enDirection = UsartDataLsbFirst,
		.enStopBit = UsartOneStopBit,
		.enParity = UsartParityNone,
		.enSampleMode = UsartSampleBit8,
		.enDetectMode = UsartStartBitFallEdge,
		.enHwFlow = UsartRtsEnable,
	};

	// clear ring buffers
	config->state.rx_buffer->clear();
	config->state.tx_buffer->clear();

	// enable peripheral clock
	PWC_Fcg1PeriphClockCmd(config->peripheral.clock_id, Enable);

	// initialize usart peripheral and set baud rate
	USART_UART_Init(config->peripheral.register_base, &stcInitCfg);
}

void usart_set_baud_rate(struct usart_config_t *config, uint32_t baud)
{

	USART_SetBaudrate(config->peripheral.register_base, baud);
}

void usart_enable(struct usart_config_t *config)
{
	// set rx IRQ
	stc_irq_regi_conf_t irqConf;
	auto reg = config->peripheral.register_base;
	if (reg == M4_USART1)
	{
		irqConf.enIRQn = IRQ_INDEX_INT_USART1_RI;
		irqConf.enIntSrc = INT_USART1_RI;
		irqConf.pfnCallback = &Usart1RxIrqCallback;
	}
	else if (reg == M4_USART2)
	{
		irqConf.enIRQn = IRQ_INDEX_INT_USART2_RI;
		irqConf.enIntSrc = INT_USART2_RI;
		irqConf.pfnCallback = &Usart2RxIrqCallback;
	}
	else if (reg == M4_USART3)
	{
		irqConf.enIRQn = IRQ_INDEX_INT_USART3_RI;
		irqConf.enIntSrc = INT_USART3_RI;
		irqConf.pfnCallback = &Usart3RxIrqCallback;
	}
	else if (reg == M4_USART4)
	{
		irqConf.enIRQn = IRQ_INDEX_INT_USART4_RI;
		irqConf.enIntSrc = INT_USART4_RI;
		irqConf.pfnCallback = &Usart4RxIrqCallback;
	}

	enIrqRegistration(&irqConf);
	NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
	NVIC_ClearPendingIRQ(irqConf.enIRQn);
	NVIC_EnableIRQ(irqConf.enIRQn);

	// set rx error IRQ
	irqConf.enIRQn = IRQ_INDEX_INT_USART1_EI;
	if (reg == M4_USART1)
	{
		irqConf.enIntSrc = INT_USART1_EI;
		irqConf.pfnCallback = &Usart1ErrIrqCallback;
	}
	else if (reg == M4_USART2)
	{
		irqConf.enIntSrc = INT_USART2_EI;
		irqConf.pfnCallback = &Usart2ErrIrqCallback;
	}
	else if (reg == M4_USART3)
	{
		irqConf.enIntSrc = INT_USART3_EI;
		irqConf.pfnCallback = &Usart3ErrIrqCallback;
	}
	else if (reg == M4_USART4)
	{
		irqConf.enIntSrc = INT_USART4_EI;
		irqConf.pfnCallback = &Usart4ErrIrqCallback;
	}
	enIrqRegistration(&irqConf);
	NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
	NVIC_ClearPendingIRQ(irqConf.enIRQn);
	NVIC_EnableIRQ(irqConf.enIRQn);

	// set tx IRQ
	irqConf.enIRQn = IRQ_INDEX_INT_USART1_TI;
	if (reg == M4_USART1)
	{
		irqConf.enIntSrc = INT_USART1_TI;
		irqConf.pfnCallback = &Usart1TxIrqCallback;
	}
	else if (reg == M4_USART2)
	{
		irqConf.enIntSrc = INT_USART2_TI;
		irqConf.pfnCallback = &Usart2TxIrqCallback;
	}
	else if (reg == M4_USART3)
	{
		irqConf.enIntSrc = INT_USART3_TI;
		irqConf.pfnCallback = &Usart3TxIrqCallback;
	}
	else if (reg == M4_USART4)
	{
		irqConf.enIntSrc = INT_USART4_TI;
		irqConf.pfnCallback = &Usart4TxIrqCallback;
	}
	enIrqRegistration(&irqConf);
	NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
	NVIC_ClearPendingIRQ(irqConf.enIRQn);
	NVIC_EnableIRQ(irqConf.enIRQn);

	// set tx complete IRQ
	irqConf.enIRQn = IRQ_INDEX_INT_USART1_TCI;
	if (reg == M4_USART1)
	{
		irqConf.enIntSrc = INT_USART1_TCI;
		irqConf.pfnCallback = &Usart1TxCmpltIrqCallback;
	}
	else if (reg == M4_USART2)
	{
		irqConf.enIntSrc = INT_USART2_TCI;
		irqConf.pfnCallback = &Usart2TxCmpltIrqCallback;
	}
	else if (reg == M4_USART3)
	{
		irqConf.enIntSrc = INT_USART3_TCI;
		irqConf.pfnCallback = &Usart3TxCmpltIrqCallback;
	}
	else if (reg == M4_USART4)
	{
		irqConf.enIntSrc = INT_USART4_TCI;
		irqConf.pfnCallback = &Usart4TxCmpltIrqCallback;
	}
	enIrqRegistration(&irqConf);
	NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
	NVIC_ClearPendingIRQ(irqConf.enIRQn);
	NVIC_EnableIRQ(irqConf.enIRQn);

	// enable IRQs
	USART_FuncCmd(M4_USART1, UsartRx, Enable);
	USART_FuncCmd(M4_USART1, UsartTx, Enable);

	USART_FuncCmd(M4_USART1, UsartRxInt, Enable);
	// USART_FuncCmd(M4_USART1, UsartTxCmpltInt, Enable);
}

void usart_disable(struct usart_config_t *config)
{
	// wait until ty buffer is empty
	while (!config->state.tx_buffer->isEmpty())
		;

	// disable uart peripheral
	USART_FuncCmd(config->peripheral.register_base, UsartTx, Disable);
	USART_FuncCmd(config->peripheral.register_base, UsartRx, Disable);

	// resign usart interrupts
	usart_irq_resign(config->interrupts.rx_data_available, "usart rx data available");
	usart_irq_resign(config->interrupts.rx_error, "usart rx error");
	usart_irq_resign(config->interrupts.tx_buffer_empty, "usart tx buffer empty");
	usart_irq_resign(config->interrupts.tx_complete, "usart tx complete");

	// deinit uart
	USART_DeInit(config->peripheral.register_base);

	// clear buffers
	usart_reset_rx(config);
	usart_reset_tx(config);
}

uint32_t usart_tx(struct usart_config_t *config, const uint8_t *buf, uint32_t len)
{

	uint32_t sentBytes = 0;
	while (sentBytes < len)
	{
		config->state.tx_buffer->push(buf);
		sentBytes++;
	}
	if (!config->state.tx_buffer->isEmpty())
	{
		USART_FuncCmd(config->peripheral.register_base, UsartTxAndTxEmptyInt, Enable);
	}

	return sentBytes;
}

uint32_t usart_rx(struct usart_config_t *config, uint8_t *buf, uint32_t len)
{
	uint32_t receivedBytes = 0;
	uint32_t errors = 0;
	while (usart_data_available(config) && receivedBytes < len)
	{
		*buf++ = usart_getc(config);
		receivedBytes++;
		if (++errors > 500)
		{
			errors = 0;
			break;
		}
	}

	return receivedBytes;
}

void usart_putudec(struct usart_config_t *config, uint32_t val)
{
	char digits[12];
	int i = 0;

	do
	{
		digits[i++] = val % 10 + '0';
		val /= 10;
	} while (val > 0);

	while (--i >= 0)
	{
		usart_putc(config, digits[i]);
	}
}

size_t usart_print(struct usart_config_t *config, const char *msg)
{
	size_t i;
	do
	{
		usart_putc(config, msg[i]);
		i++;
	} while (msg[i] != '\0' && i < PRINTF_BUFFER_SIZE);

	return i;
}

char printf_buffer[PRINTF_BUFFER_SIZE];

size_t usart_printf(struct usart_config_t *config, const char *fmt, ...)
{
	// format the string into buffer
	va_list args;
	va_start(args, fmt);
	vsnprintf(printf_buffer, PRINTF_BUFFER_SIZE, fmt, args);
	va_end(args);

	return usart_print(config, printf_buffer);
}
