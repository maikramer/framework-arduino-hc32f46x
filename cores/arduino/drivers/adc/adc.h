#pragma once


#include "hc32_ddl.h"


#ifdef __cplusplus
extern "C"
{
#endif


#define BOARD_ADC_CH0_PORT       (PortC)
#define BOARD_ADC_CH0_PIN        (Pin00)
#define BOARD_ADC_CH0_CH         (ADC1_CH10)

#define BOARD_ADC_CH1_PORT       (PortC)
#define BOARD_ADC_CH1_PIN        (Pin01)
#define BOARD_ADC_CH1_CH         (ADC1_CH11)

#define BOARD_ADC_CH2_PORT       (PortC)
#define BOARD_ADC_CH2_PIN        (Pin02)
#define BOARD_ADC_CH2_CH         (ADC1_CH12)

/* Timer definition for this example. */
#define TMR_UNIT                 (M4_TMR02)

#define ADC_CHANNEL_COUNT 2u
#define ADC1_SA_CHANNEL (ADC1_CH14 | ADC1_CH15)
#define ADC1_SA_CHANNEL_COUNT (ADC_CHANNEL_COUNT)

// ADC irq flag bit mask
#define ADC1_SA_DMA_IRQ_BIT (1ul << 0u)

//
// ADC device definition
//
typedef struct adc_device_t {
    __IO uint32_t
    HAL_AdcDmaIrqFlag;
    __IO uint16_t
    HAL_adc_results[ADC1_CH_COUNT];

    M4_ADC_TypeDef *regs;
    __IO uint32_t
    PeriphClock;
    __IO uint32_t
    Channel;

    M4_DMA_TypeDef *DMARegs;
    __IO uint32_t
    DMAPeriphClock;
    __IO uint8_t
    DMAChannel;
    __IO en_event_src_t
    DMAenSrc;
} adc_device_t;

extern adc_device_t ADC1_device;
extern struct adc_device_t *ADC1;
extern stc_adc_init_t stcAdcInit;

extern uint16_t AdcCH0SampleBuf[256];
extern uint16_t AdcCH1SampleBuf[256];
extern uint16_t AdcCH2SampleBuf[256];

extern uint32_t AdcCH0Value;
extern uint32_t AdcCH1Value;
extern uint32_t AdcCH2Value;

extern uint16_t g_adc_value[3];
extern uint8_t g_adc_idx;

void adc_init(void);

void adc_pin_init(void);

void AdcClockConfig(void);

void AdcInitConfig(void);

void AdcChannelConfig(void);

void AdcTriggerConfig(void);


void AdcSetChannelPinMode(const M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode);

void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode);

void adc_dma_config(void);


void BSP_DMA2CH0_TcIrqHander(void);

void BSP_DMA2CH1_TcIrqHander(void);

void BSP_DMA2CH2_TcIrqHander(void);


void AdcConfig(void);

#ifdef __cplusplus
}
#endif