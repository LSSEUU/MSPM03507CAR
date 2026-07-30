#ifndef GRAYSCALE_CONFIG_H
#define GRAYSCALE_CONFIG_H

/* OUT.8IR.B I2C line sensor compatibility configuration. */
#ifndef GRAYSCALE_CALIBRATION_VERIFIED
#define GRAYSCALE_CALIBRATION_VERIFIED          1U
#endif

#define GRAYSCALE_CALIBRATION_MIN_SPAN_ADC    512U
#define GRAYSCALE_I2C_POLL_PERIOD_MS           20U
#define GRAYSCALE_STARTUP_GRACE_MS           3000U
#define GRAYSCALE_PROXY_LINE_ADC                1U
#define GRAYSCALE_PROXY_BACKGROUND_ADC       4094U

/* The installed module has IR1 on the vehicle's left-hand side. */
#ifndef GRAYSCALE_IR1_ON_RIGHT
#define GRAYSCALE_IR1_ON_RIGHT                  0U
#endif

#if ((GRAYSCALE_IR1_ON_RIGHT != 0U) && (GRAYSCALE_IR1_ON_RIGHT != 1U))
#error "GRAYSCALE_IR1_ON_RIGHT must be 0 or 1"
#endif

#ifndef GRAYSCALE_WHITE_ADC_VALUES
#define GRAYSCALE_WHITE_ADC_VALUES \
    { 4094U, 4094U, 4094U, 4094U, 4094U, 4094U, 4094U, 4094U }
#endif

#ifndef GRAYSCALE_BLACK_ADC_VALUES
#define GRAYSCALE_BLACK_ADC_VALUES \
    { 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U }
#endif

#endif
