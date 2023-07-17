#include "bsp_pwm.h"
#include "sysclock/sysclock.h"

#define FAN_PWM_FREQUENCY 20000UL //20khz
#define BEEP_PWM_FREQUENCY 400UL  //400hz

uint16_t fan_get_period(en_timera_clk_div prescale) {
    auto pclk1 = SYSTEM_CLOCK_FREQUENCIES.pclk1;
    return ((pclk1 / (1UL << prescale)) / FAN_PWM_FREQUENCY) - 1;
}

void fan_pwm_init(void) {
    hal_fan_pwm_init(0);
    hal_fan_pwm_init(2);
}

void hal_fan_pwm_init(uint8_t fan) {
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcTimerCompareInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA2 | PWC_FCG2_PERIPH_TIMA3 | PWC_FCG2_PERIPH_TIMA4,
                           Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    M4_TMRA_TypeDef *tim_base;
    en_timera_channel_t tim_ch;
    en_port_t tim_port;
    en_pin_t tim_pin;
    en_port_func_t tim_func;

    switch (fan) {
        case 0:
            tim_base = BOARD_PWM_CH0_BASE;
            tim_ch = BOARD_PWM_CH0_CH;
            tim_port = BOARD_PWM_CH0_PORT;
            tim_pin = BOARD_PWM_CH0_PIN;
            tim_func = BOARD_PWM_CH0_FUNC;
            break;

        case 1:
            tim_base = BOARD_PWM_CH1_BASE;
            tim_ch = BOARD_PWM_CH1_CH;
            tim_port = BOARD_PWM_CH1_PORT;
            tim_pin = BOARD_PWM_CH1_PIN;
            tim_func = BOARD_PWM_CH1_FUNC;
            break;

        case 2:
            tim_base = BOARD_PWM_CH2_BASE;
            tim_ch = BOARD_PWM_CH2_CH;
            tim_port = BOARD_PWM_CH2_PORT;
            tim_pin = BOARD_PWM_CH2_PIN;
            tim_func = BOARD_PWM_CH2_FUNC;
            break;

        default:
            return;
            break;
    }

    /* Configuration TIMERA compare pin */
    PORT_SetFunc(tim_port, tim_pin, tim_func, Disable);
    stcTimeraInit.enClkDiv = TimeraPclkDiv2;
    // PWM = 20K

    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = fan_get_period(stcTimeraInit.enClkDiv);

    TIMERA_BaseInit(tim_base, &stcTimeraInit);
    TIMERA_IrqCmd(tim_base, TimeraIrqOverflow, Disable);

    stcTimerCompareInit.u16CompareVal = 0;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputKeep;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;

    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputLow;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputLow;

    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Disable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;

    TIMERA_CompareInit(tim_base, tim_ch, &stcTimerCompareInit);
    TIMERA_CompareCmd(tim_base, tim_ch, Enable);

    TIMERA_Cmd(tim_base, Enable);

    TIMERA_SetCompareValue(tim_base, tim_ch, 0);
    TIMERA_SpecifyOutputSta(tim_base, tim_ch, TimeraSpecifyOutputLow);
}

// ratio is 0~255
void fan_pwm_set_ratio(uint8_t fan, uint16_t ratio) {
    M4_TMRA_TypeDef *tim_base;
    en_timera_channel_t tim_ch;

    switch (fan) {
        case 0:
            tim_base = BOARD_PWM_CH0_BASE;
            tim_ch = BOARD_PWM_CH0_CH;
            break;

        case 1:
            tim_base = BOARD_PWM_CH1_BASE;
            tim_ch = BOARD_PWM_CH1_CH;
            break;

        case 2:
            tim_base = BOARD_PWM_CH2_BASE;
            tim_ch = BOARD_PWM_CH2_CH;
            break;

        default:
            return;
            break;
    }

    uint16_t ped = TIMERA_GetPeriodValue(tim_base);

    uint16_t compareValue = (uint16_t) ped * (ratio /256.0f);

    TIMERA_SetCompareValue(tim_base, tim_ch, compareValue);

    if (compareValue <= 10) {
        TIMERA_SpecifyOutputSta(tim_base, tim_ch, TimeraSpecifyOutputLow);
    } else if (compareValue >= ped - 5) {
        TIMERA_SpecifyOutputSta(tim_base, tim_ch, TimeraSpecifyOutputHigh);
    } else {
        TIMERA_SpecifyOutputSta(tim_base, tim_ch, TimeraSpecifyOutputInvalid);
    }
}

void beep_pwm_init(void) {
    uint32_t period;

    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcTimerCompareInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA3, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    PORT_SetFunc(BOARD_PWM_CH3_PORT, BOARD_PWM_CH3_PIN, BOARD_PWM_CH3_FUNC, Disable);
    stcTimeraInit.enClkDiv = TimeraPclkDiv8;
    auto pclk1 = SYSTEM_CLOCK_FREQUENCIES.pclk1;
    period = ((pclk1 / (1 << stcTimeraInit.enClkDiv)) / BEEP_PWM_FREQUENCY) - 1;

    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = period;

    TIMERA_BaseInit(BOARD_PWM_CH3_BASE, &stcTimeraInit);
    TIMERA_IrqCmd(BOARD_PWM_CH3_BASE, TimeraIrqOverflow, Disable);

    stcTimerCompareInit.u16CompareVal = period * 0.1;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputKeep;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;

    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputLow;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputLow;

    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Disable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;

    TIMERA_CompareInit(BOARD_PWM_CH3_BASE, BOARD_PWM_CH3_CH, &stcTimerCompareInit);
    TIMERA_CompareCmd(BOARD_PWM_CH3_BASE, BOARD_PWM_CH3_CH, Enable);

    TIMERA_SetCompareValue(BOARD_PWM_CH3_BASE, BOARD_PWM_CH3_CH, period * 0.1);
    TIMERA_SpecifyOutputSta(BOARD_PWM_CH3_BASE, BOARD_PWM_CH3_CH, TimeraSpecifyOutputInvalid);

}

void beep_pwm_set_frequency(uint32_t frequency, uint8_t ratio) {
    uint16_t period = 0, compareValue = 0;

    auto pclk1 = SYSTEM_CLOCK_FREQUENCIES.pclk1;
    uint32_t div = 1 << M4_TMRA3->BCSTR_f.CKDIV;
    period = ((pclk1 / div) / frequency);
    M4_TMRA3->PERAR = period;

    compareValue = (uint32_t)(ratio * period / 100.0f);
    TIMERA_SetCompareValue(M4_TMRA3, TimeraCh2, compareValue);
    TIMERA_SpecifyOutputSta(M4_TMRA3, TimeraCh2, TimeraSpecifyOutputInvalid);

    TIMERA_Cmd(BOARD_PWM_CH3_BASE, Enable);
}

void beep_pwm_stop(void) {
    TIMERA_Cmd(BOARD_PWM_CH3_BASE, Disable);
}


