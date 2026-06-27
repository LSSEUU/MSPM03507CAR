#include "ti_msp_dl_config.h"

SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_PWM_0_init();
}



SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Extend_reset(UART_2_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_UART_Extend_enablePower(UART_2_INST);
    DL_TimerA_enablePower(PWM_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(AIN_AIN1_IOMUX);
    DL_GPIO_initDigitalOutput(AIN_AIN2_IOMUX);
    DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_enableOutput(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);

    DL_GPIO_initDigitalOutput(BIN_BIN1_IOMUX);
    DL_GPIO_initDigitalOutput(BIN_BIN2_IOMUX);
    DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    DL_GPIO_enableOutput(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);

    DL_GPIO_initDigitalInput(ENC_A_E1A_IOMUX);
    DL_GPIO_initDigitalInput(ENC_A_E1B_IOMUX);
    DL_GPIO_setLowerPinsPolarity(ENC_A_PORT,
        DL_GPIO_PIN_8_EDGE_RISE | DL_GPIO_PIN_9_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);
    DL_GPIO_enableInterrupt(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);

    DL_GPIO_initDigitalInput(ENC_B_E2A_IOMUX);
    DL_GPIO_initDigitalInput(ENC_B_E2B_IOMUX);
    DL_GPIO_setLowerPinsPolarity(ENC_B_PORT,
        DL_GPIO_PIN_8_EDGE_RISE | DL_GPIO_PIN_9_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENC_B_PORT,
        ENC_B_E2A_PIN | ENC_B_E2B_PIN);
    DL_GPIO_enableInterrupt(ENC_B_PORT,
        ENC_B_E2A_PIN | ENC_B_E2B_PIN);

    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
}

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST,
        (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);
    DL_UART_Main_init(UART_0_INST,
        (DL_UART_Main_Config *) &gUART_0Config);
    DL_UART_Main_setOversampling(UART_0_INST,
        DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST,
        UART_0_IBRD_32_MHZ_115200_BAUD,
        UART_0_FBRD_32_MHZ_115200_BAUD);
    DL_UART_Main_enableFIFOs(UART_0_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_0_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enable(UART_0_INST);
}

static const DL_UART_Main_ClockConfig gUART_2ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_2Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_2_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_2_TX_IOMUX, GPIO_UART_2_TX_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_2_RX_IOMUX, GPIO_UART_2_RX_IOMUX_FUNC);

    DL_UART_Main_setClockConfig(UART_2_INST,
        (DL_UART_Main_ClockConfig *) &gUART_2ClockConfig);
    DL_UART_Main_init(UART_2_INST,
        (DL_UART_Main_Config *) &gUART_2Config);
    DL_UART_Main_setOversampling(UART_2_INST,
        DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_2_INST,
        UART_2_IBRD_32_MHZ_9600_BAUD,
        UART_2_FBRD_32_MHZ_9600_BAUD);
    DL_UART_Main_enableFIFOs(UART_2_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_2_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enable(UART_2_INST);
}

static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0,
};

static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode   = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period    = 1000,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void)
{
    DL_Timer_setClockConfig(PWM_0_INST,
        (DL_Timer_ClockConfig *) &gPWM_0ClockConfig);
    DL_Timer_initPWMMode(PWM_0_INST,
        (DL_Timer_PWMConfig *) &gPWM_0Config);

    DL_Timer_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,
        DL_TIMER_CC_0_INDEX);

    DL_Timer_setCaptureCompareOutCtl(PWM_0_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,
        DL_TIMER_CC_1_INDEX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX,
        GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX,
        GPIO_PWM_0_C1_IOMUX_FUNC);

    DL_Timer_enableClock(PWM_0_INST);
    DL_Timer_setCCPDirection(PWM_0_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_Timer_startCounter(PWM_0_INST);
}

static const DL_TimerG_ClockConfig gTIMER_ENC_ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

static const DL_TimerG_TimerConfig gTIMER_ENC_TimerConfig = {
    .period     = TIMER_ENC_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_ENC_init(void)
{
    DL_TimerG_enablePower(TIMER_ENC_INST);
    DL_TimerG_setClockConfig(TIMER_ENC_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_ENC_ClockConfig);
    DL_TimerG_initTimerMode(TIMER_ENC_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_ENC_TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_ENC_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_ENC_INST);
}


