#ifndef GRAYSCALE_CONFIG_H
#define GRAYSCALE_CONFIG_H

/*
 * Commissioning lock for the Ganwei analog eight-channel sensor.
 *
 * The arrays below were collected after correcting AD0 to PB1/H13-6, using
 * 64 valid scans over the installed white background and 64 over the track's
 * black material at the final sensor height. Line motion is rejected if this
 * lock is cleared or any channel span becomes invalid.
 */
#ifndef GRAYSCALE_CALIBRATION_VERIFIED
#define GRAYSCALE_CALIBRATION_VERIFIED          1U
#endif

#define GRAYSCALE_CALIBRATION_MIN_SPAN_ADC    512U

#ifndef GRAYSCALE_WHITE_ADC_VALUES
#define GRAYSCALE_WHITE_ADC_VALUES \
    { 2804U, 2548U, 3026U, 2709U, 1112U, 3063U, 3071U, 2253U }
#endif

#ifndef GRAYSCALE_BLACK_ADC_VALUES
#define GRAYSCALE_BLACK_ADC_VALUES \
    { 102U, 106U, 108U, 109U, 109U, 106U, 282U, 109U }
#endif

#endif
