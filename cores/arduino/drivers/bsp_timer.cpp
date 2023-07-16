#include "hc32f460.h"

#include "bsp_timer.h"
#include "usart/usart.h"
#include "SoftwareSerial.h"
#include <stdio.h>

#define IRQ_INDEX_INT_TMR01_GCMA        Int019_IRQn
#define IRQ_INDEX_INT_TMR01_GCMB        Int020_IRQn

#define IRQ_INDEX_INT_TMR02_GCMA        Int021_IRQn
#define IRQ_INDEX_INT_TMR02_GCMB        Int022_IRQn

#define IRQ_INDEX_INT_TMR41_GCMB        Int023_IRQn
#define IRQ_INDEX_INT_TMR42_GCMB        Int024_IRQn

/*!< Parameter validity check for Timer4 unit  */
#define IS_VALID_TIMER4(__TMRx__) \
  ((M4_TMR41 == (__TMRx__)) ||    \
   (M4_TMR42 == (__TMRx__)) ||    \
   (M4_TMR43 == (__TMRx__)))

static en_result_t TIMER4_CNT_Load(M4_TMR4_TypeDef *TMR4x, stc_timer4_cnt_init_t *pstcInitCfg) {
    en_result_t enRet = ErrorInvalidParameter;

    if ((IS_VALID_TIMER4(TMR4x)) && (NULL != pstcInitCfg)) {

        pstcInitCfg->u16Cycle = TMR4x->CPSR;

        pstcInitCfg->enCntMode = (en_timer4_cnt_mode_t) TMR4x->CCSR_f.MODE;
        pstcInitCfg->enClk = (en_timer4_cnt_clk_t) TMR4x->CCSR_f.ECKEN;
        pstcInitCfg->enClkDiv = (en_timer4_cnt_clk_div_t) TMR4x->CCSR_f.CKDIV;
        pstcInitCfg->enBufferCmd = (en_functional_state_t) TMR4x->CCSR_f.BUFEN;
        pstcInitCfg->enZeroIntCmd = (en_functional_state_t) TMR4x->CCSR_f.IRQZEN;
        pstcInitCfg->enPeakIntCmd = (en_functional_state_t) TMR4x->CCSR_f.IRQPEN;

        pstcInitCfg->enZeroIntMsk = (en_timer4_cnt_int_mask_t) TMR4x->CVPR_f.ZIM;
        pstcInitCfg->enPeakIntMsk = (en_timer4_cnt_int_mask_t) TMR4x->CVPR_f.PIM;

        enRet = Ok;
    }

    return enRet;
}

extern void Timer01B_CallBack(void);

void timer01B_init(void) {
    stc_tim0_base_init_t timerConf;

    MEM_ZERO_STRUCT(timerConf);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM01, Enable);

    timerConf.Tim0_CounterMode = Tim0_Sync;
    timerConf.Tim0_SyncClockSource = Tim0_Pclk1;
    timerConf.Tim0_ClockDivision = Tim0_ClkDiv1024;
    timerConf.Tim0_CmpValue = (uint16_t)(SYSTEM_CLOCK_FREQUENCIES.pclk1 / 1024ul / 1000);
    TIMER0_BaseInit(M4_TMR01, Tim0_ChannelB, &timerConf);

    stc_irq_regi_conf_t stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TMR01_GCMB;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = INT_TMR01_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer01B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    TIMER0_IntCmd(M4_TMR01, Tim0_ChannelB, Enable);

//    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Enable);
}

void timer01B_set_overflow(uint16_t ms) {
    M4_TMR01->CMPBR_f.CMPB = (uint16_t)(((SYSTEM_CLOCK_FREQUENCIES.pclk1 / 1024ul) / 1000) * ms);

    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Enable);
}

void timer01B_enable(void) {
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Enable);
}

void timer01B_disable(void) {
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Disable);
    M4_TMR01->CNTBR = 0;
}

extern void Timer02B_CallBack(void);

void timer02B_init(void) {
    stc_tim0_base_init_t timerConf;

    MEM_ZERO_STRUCT(timerConf);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable);

    timerConf.Tim0_CounterMode = Tim0_Sync;
    timerConf.Tim0_SyncClockSource = Tim0_Pclk1;
    timerConf.Tim0_ClockDivision = Tim0_ClkDiv0;
    timerConf.Tim0_CmpValue = (uint16_t)(SYSTEM_CLOCK_FREQUENCIES.pclk1 / (SS_BAUDRATE * 3));
    TIMER0_BaseInit(TMR_SSERIAL_UNIT, Tim0_ChannelB, &timerConf);

    stc_irq_regi_conf_t stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TMR02_GCMB;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = INT_TMR02_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer02B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    TIMER0_IntCmd(TMR_SSERIAL_UNIT, Tim0_ChannelB, Enable);

    TIMER0_Cmd(TMR_SSERIAL_UNIT, Tim0_ChannelB, Enable);
}

void timer41_init(void) {
    stc_irq_regi_conf_t irqConf;
    MEM_ZERO_STRUCT(irqConf);

    stc_timer4_cnt_init_t stcCntInit;
    MEM_ZERO_STRUCT(stcCntInit);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41, Enable);

    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;
    stcCntInit.u16Cycle =
            (SYSTEM_CLOCK_FREQUENCIES.pclk1 / (1 << stcCntInit.enClkDiv)) / 1000ul;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Enable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(M4_TMR41, &stcCntInit);

    // register timer interrupt on choosen irq
    irqConf.enIRQn = IRQ_INDEX_INT_TMR41_GCMB;
    irqConf.enIntSrc = INT_TMR41_GUDF;

    // set irq handler
    irqConf.pfnCallback = &timer41_zero_match_irq_cb;

    // register the irq
    enIrqRegistration(&irqConf);

    // clear pending irq, set priority and enable
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(irqConf.enIRQn);

    TIMER4_CNT_ClearCountVal(M4_TMR41);
    TIMER4_CNT_Start(M4_TMR41);
}

void timer41_set_frequency(const uint32_t frequency) {
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);

    TIMER4_CNT_Load(M4_TMR41, &stcCntInit);
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;
    stcCntInit.u16Cycle = SYSTEM_CLOCK_FREQUENCIES.pclk1 / (1 << stcCntInit.enClkDiv) / frequency;

    TIMER4_CNT_Init(M4_TMR41, &stcCntInit);

    TIMER4_CNT_Start(M4_TMR41);
}

uint16_t timer41_get_count() {
    return TIMER4_CNT_GetCountVal(M4_TMR41);
}

#define TIMER42_CNT_CYCLE_VAL 65534

void timer42_init(void) {
    stc_irq_regi_conf_t irqConf;
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(irqConf);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM42, Enable);

    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;
    stcCntInit.u16Cycle = TIMER42_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Enable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(M4_TMR42, &stcCntInit);

    irqConf.enIRQn = IRQ_INDEX_INT_TMR42_GCMB;
    irqConf.pfnCallback = &timer42_zero_match_irq_cb;
    irqConf.enIntSrc = INT_TMR42_GUDF;
    enIrqRegistration(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIORITY_00);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);

    TIMER4_CNT_ClearCountVal(M4_TMR42);
    TIMER4_CNT_Start(M4_TMR42);
}

void timer42_init_check(void) {
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);

    TIMER4_CNT_Load(M4_TMR42, &stcCntInit);
}

void timer42_set_frequency(const uint32_t frequency) {
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);

    TIMER4_CNT_Load(M4_TMR42, &stcCntInit);
    stcCntInit.u16Cycle = SYSTEM_CLOCK_FREQUENCIES.pclk1 / (1 << stcCntInit.enClkDiv) / frequency;

    TIMER4_CNT_Init(M4_TMR42, &stcCntInit);

    TIMER4_CNT_Start(M4_TMR42);
}

void timer42_irq_ctrl(en_functional_state_t state) {
    if (state == Enable) {
        NVIC_EnableIRQ(IRQ_INDEX_INT_TMR42_GCMB);
    } else if (state == Disable) {
        NVIC_DisableIRQ(IRQ_INDEX_INT_TMR42_GCMB);
    }
}

bool timer42_irq_get() {
    return (bool) NVIC_GetEnableIRQ(IRQ_INDEX_INT_TMR42_GCMB);
}

bool timer42_set_compare(const uint16_t compare) {
    TIMER4_CNT_SetCycleVal(M4_TMR42, compare);
    return true;
}

uint16_t timer42_get_count() {
    return TIMER4_CNT_GetCountVal(M4_TMR42);
}

