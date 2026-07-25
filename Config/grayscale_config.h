#ifndef GRAYSCALE_CONFIG_H
#define GRAYSCALE_CONFIG_H

/*
 * Commissioning lock for the Ganwei analog eight-channel sensor.
 *
 * Keep this at 0 until scripts/calibrate_grayscale.py has collected 64 valid
 * scans over the installed white background and black line, and the emitted
 * arrays have replaced the placeholders below. Line motion is rejected while
 * the lock is 0 or any channel span is invalid.
 */
#ifndef GRAYSCALE_CALIBRATION_VERIFIED
#define GRAYSCALE_CALIBRATION_VERIFIED          1U
#endif

#define GRAYSCALE_CALIBRATION_MIN_SPAN_ADC    512U

#ifndef GRAYSCALE_WHITE_ADC_VALUES
#define GRAYSCALE_WHITE_ADC_VALUES \
    { 2419U, 2420U, 2886U, 2887U, 940U, 941U, 3006U, 3007U }
#endif

#ifndef GRAYSCALE_BLACK_ADC_VALUES
#define GRAYSCALE_BLACK_ADC_VALUES \
    { 72U, 73U, 79U, 80U, 79U, 79U, 142U, 142U }
#endif

#endif
