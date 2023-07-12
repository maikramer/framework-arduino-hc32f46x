#include "wiring_digital.h"
#include "drivers/gpio/gpio.h"
#include "drivers/adc/adc.h"
#include "wiring_constants.h"

void pinMode(uint32_t dwPin, uint32_t dwMode)
{
    if (dwPin >= BOARD_NR_GPIO_PINS)
    {
        return;
    }

    // if pin has ADC channel, configure ADC according to pin mode
    adc_device_t *adc_device = PIN_MAP[dwPin].adc_device;
    uint8_t adc_channel = PIN_MAP[dwPin].adc_channel;
    if (adc_device != NULL && adc_channel != ADC_PIN_INVALID)
    {
        // is a valid ADC pin
        if (dwMode == INPUT_ANALOG)
        {
            // initialize adc device (if already initialized, this will do nothing)
//            adc_init();

            // enable ADC channel
//            adc_enable_channel(adc_device, adc_channel);
        }
        else
        {
            // disable ADC channel
//            adc_disable_channel(adc_device, adc_channel);
        }
    }

    // build pin configuration
    stc_port_init_t pinConf;
    MEM_ZERO_STRUCT(pinConf);
    switch (dwMode)
    {
    case INPUT:
        pinConf.enPinMode = Pin_Mode_In;
        break;
    case INPUT_PULLUP:
        pinConf.enPinMode = Pin_Mode_In;
        pinConf.enPullUp = Enable;
        break;
    case INPUT_ANALOG:
        pinConf.enPinMode = Pin_Mode_Ana;
        break;
    case OUTPUT:
        pinConf.enPinMode = Pin_Mode_Out;
        break;
    default:
        return;
    }

    // set pin function and config
    GPIO_SetFunc(dwPin, Func_Gpio, Enable);
    GPIO_Init(dwPin, &pinConf);
}

uint32_t getPinMode(uint32_t dwPin)
{
    if (dwPin >= BOARD_NR_GPIO_PINS)
    {
        return INPUT_FLOATING;
    }

    // read pin configuration
    stc_port_init_t pinConf;
    GPIO_GetConfig(dwPin, &pinConf);
    switch (pinConf.enPinMode)
    {
    case Pin_Mode_Out:
        return OUTPUT;
    case Pin_Mode_In:
        return (pinConf.enPullUp == Enable) ? INPUT_PULLUP : INPUT;
    case Pin_Mode_Ana:
        return INPUT_ANALOG;
    default:
        return INPUT_FLOATING;
    }
}

void digitalWrite(uint32_t dwPin, uint32_t dwVal)
{
    if (dwPin >= BOARD_NR_GPIO_PINS)
    {
        return;
    }

    if (dwVal == HIGH)
    {
        GPIO_SetBits(dwPin);
    }
    else
    {
        GPIO_ResetBits(dwPin);
    }
}

int digitalRead(uint32_t ulPin)
{
    if (ulPin >= BOARD_NR_GPIO_PINS)
    {
        return LOW;
    }

    return GPIO_GetBit(ulPin) ? HIGH : LOW;
}
