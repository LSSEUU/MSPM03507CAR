#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

/*
 * Frozen board configuration for the Tianmengxing MSPM0G3507 carrier.
 *
 * This pair of ti_msp_dl_config.c/.h files is the repository source of truth.
 * It was originally produced by SysConfig and then extended for the carrier.
 * Do not overwrite it from an unrelated LaunchPad .syscfg file.  Pin and
 * peripheral invariants are guarded by Tests/test_board_configuration.py.
 */
#define FROZEN_BOARD_CONFIG_REVISION                         (20260729U)

#define CONFIG_LP_MSPM0G3507
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_STARTUP_DELAY                                                (16)

#define CPUCLK_FREQ                                                     32000000

#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)

/* Dedicated camera/vision link: PB6 MCU_TX, PB7 MCU_RX, 115200 8N1. */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_TX_IOMUX                                      (IOMUX_PINCM23)
#define GPIO_UART_1_TX_IOMUX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_RX_IOMUX                                      (IOMUX_PINCM24)
#define GPIO_UART_1_RX_IOMUX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_1_FBRD_32_MHZ_115200_BAUD                                      (23)

/* ── UART2 (JY61P 陀螺仪) ── */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                           32000000
#define GPIO_UART_2_TX_PORT                                                GPIOA
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_21
#define GPIO_UART_2_TX_IOMUX                                     (IOMUX_PINCM46)
#define GPIO_UART_2_TX_IOMUX_FUNC                     IOMUX_PINCM46_PF_UART2_TX
#define GPIO_UART_2_RX_PORT                                                GPIOA
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_22
#define GPIO_UART_2_RX_IOMUX                                     (IOMUX_PINCM47)
#define GPIO_UART_2_RX_IOMUX_FUNC                     IOMUX_PINCM47_PF_UART2_RX
#define UART_2_BAUD_RATE                                                  (9600)
#define UART_2_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_2_FBRD_32_MHZ_9600_BAUD                                       (21)

/* Dedicated ZDT closed-loop stepper link: PB12 MCU_TX, PB13 MCU_RX. */
#define UART_3_INST                                                        UART3
#define UART_3_INST_FREQUENCY                                           32000000
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_TX_PORT                                                GPIOB
#define GPIO_UART_3_TX_PIN                                        DL_GPIO_PIN_12
#define GPIO_UART_3_TX_IOMUX                                      (IOMUX_PINCM29)
#define GPIO_UART_3_TX_IOMUX_FUNC                      IOMUX_PINCM29_PF_UART3_TX
#define GPIO_UART_3_RX_PORT                                                GPIOB
#define GPIO_UART_3_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_UART_3_RX_IOMUX                                      (IOMUX_PINCM30)
#define GPIO_UART_3_RX_IOMUX_FUNC                      IOMUX_PINCM30_PF_UART3_RX
#define UART_3_BAUD_RATE                                                (115200)
#define UART_3_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_3_FBRD_32_MHZ_115200_BAUD                                      (23)

#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000

#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_2
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM15)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM15_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX

#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_3
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

#define AIN_PORT                                                         (GPIOA)
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_14)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM36)
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM35)

#define BIN_PORT                                                         (GPIOA)
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_16)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM38)
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_17)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM39)

#define ENC_A_PORT                                                       (GPIOA)
#define ENC_A_E1A_PIN                                            (DL_GPIO_PIN_8)
#define ENC_A_E1A_IOMUX                                         (IOMUX_PINCM19)
#define ENC_A_E1B_PIN                                            (DL_GPIO_PIN_9)
#define ENC_A_E1B_IOMUX                                         (IOMUX_PINCM20)

#define ENC_B_PORT                                                       (GPIOB)
#define ENC_B_E2A_PIN                                            (DL_GPIO_PIN_9)
#define ENC_B_E2A_IOMUX                                         (IOMUX_PINCM26)
#define ENC_B_E2B_PIN                                            (DL_GPIO_PIN_8)
#define ENC_B_E2B_IOMUX                                         (IOMUX_PINCM25)

/* ── 八路灰度传感器 ── */
#define GRAY_AD0_PORT                                                   (GPIOB)
#define GRAY_AD0_PIN                                             (DL_GPIO_PIN_1)
#define GRAY_AD0_IOMUX                                          (IOMUX_PINCM13)
#define GRAY_AD1_PORT                                                   (GPIOA)
#define GRAY_AD1_PIN                                            (DL_GPIO_PIN_24)
#define GRAY_AD1_IOMUX                                          (IOMUX_PINCM54)
#define GRAY_AD2_PORT                                                   (GPIOA)
#define GRAY_AD2_PIN                                            (DL_GPIO_PIN_26)
#define GRAY_AD2_IOMUX                                          (IOMUX_PINCM59)
#define GRAY_OUT_PORT                                                   (GPIOA)
#define GRAY_OUT_PIN                                            (DL_GPIO_PIN_25)
#define GRAY_OUT_IOMUX                                          (IOMUX_PINCM55)
#define GRAY_ADC_INST                                                    (ADC0)
#define GRAY_ADC_MEM_IDX                                      (DL_ADC12_MEM_IDX_0)
#define GRAY_ADC_INPUT_CHAN                                (DL_ADC12_INPUT_CHAN_2)

/* SSD1306 connector: software I2C, address is probed at 0x3C/0x3D. */
#define OLED_SDA_PORT                                                   (GPIOA)
#define OLED_SDA_PIN                                            (DL_GPIO_PIN_28)
#define OLED_SDA_IOMUX                                          (IOMUX_PINCM3)
#define OLED_SCL_PORT                                                   (GPIOA)
#define OLED_SCL_PIN                                            (DL_GPIO_PIN_31)
#define OLED_SCL_IOMUX                                          (IOMUX_PINCM6)

/* Carrier KEY1: external 4.7 kOhm pull-down, pressed level is high. */
#define KEY1_PORT                                                       (GPIOB)
#define KEY1_PIN                                                (DL_GPIO_PIN_18)
#define KEY1_IOMUX                                              (IOMUX_PINCM44)

/* Latest populated carrier routes the active-high buzzer SIG to PA12. */
#define BUZZER_PORT                                                     (GPIOA)
#define BUZZER_PIN                                              (DL_GPIO_PIN_12)
#define BUZZER_IOMUX                                            (IOMUX_PINCM34)

/* No MCU-controlled status LED is populated on the verified carrier. */
#define BOARD_STATUS_LED_COUNT                                             (0U)

#define TIMER_ENC_INST                                                    TIMG12
#define TIMER_ENC_INST_IRQHandler                             TIMG12_IRQHandler
#define TIMER_ENC_INST_INT_IRQN                               (TIMG12_INT_IRQn)
#define TIMER_ENC_INST_LOAD_VALUE                                       (31999)

/* VS1838B NEC receiver on rotary-encoder connector pin 4 (PB11). */
#define IR_CAPTURE_INST                                                   TIMG0
#define IR_CAPTURE_INST_IRQHandler                            TIMG0_IRQHandler
#define IR_CAPTURE_INST_INT_IRQN                              (TIMG0_INT_IRQn)
#define IR_CAPTURE_INST_LOAD_VALUE                                  (65535U)
#define IR_CAPTURE_CC_INDEX                               DL_TIMER_CC_1_INDEX
#define IR_CAPTURE_PORT                                                   GPIOB
#define IR_CAPTURE_PIN                                           DL_GPIO_PIN_11
#define IR_CAPTURE_IOMUX                                         IOMUX_PINCM28
#define IR_CAPTURE_IOMUX_FUNC                    IOMUX_PINCM28_PF_TIMG0_CCP1
#define IR_CAPTURE_TICKS_PER_US                                         (4U)

void SYSCFG_DL_init(void);
void SYSCFG_DL_TIMER_ENC_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_UART_3_init(void);
void SYSCFG_DL_GRAY_ADC_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_IR_CAPTURE_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif
