/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-bmp280 BMP280 Driver
 * @{
 *
 * \file
 * Header file for BMP280 sensor driver
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 *
 */


#ifndef BMP280_SENSOR_H_
#define BMP280_SENSOR_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"
#include <stdint.h>

#define BMP280_SENSOR         "BMP280"

#define BMP280_NB_CAL_REG     24

#define BMP280_PWR_PORT_BASE  GPIO_PORT_TO_BASE(BMP280_PWR_PORT)
#define BMP280_PWR_PIN_MASK   GPIO_PIN_MASK(BMP280_PWR_PIN)

/* -----------------------------------------------------------------*/
/*				/!\ No not alter values below /!\					*/
/* -----------------------------------------------------------------*/
/**
 * \name Chip registers
 *
 * @{
 */
#define BMP280_CMD_USER_WRITE   0xE6	// Write user register
#define BMP280_CMD_USER_READ    0xE7	// Read user register


#define BMP280_STATUS           0xF3
#define BMP280_CTRL_MEAS        0xF4
#define BMP280_CONFIG           0xF5
#define BMP280_RESET            0xE0
#define BMP280_ID               0xD0	// must have data 0x58
#define BMP280_ID_DATA          0x58
#define BMP280_PRESS_MSB        0xF7
#define BMP280_PRESS_LSB        0xF8
#define BMP280_PRESS_XLSB       0xF9
#define BMP280_TEMP_MSB         0xFA
#define BMP280_TEMP_LSB         0xFB
#define BMP280_TEMP_XLSB        0xFC
// 24 calibration registers
#define BMP280_CALIB_0          0x88
#define BMP280_CALIB_23         0x9F

/** @} */
/************************************************/
/**\name POWER MODE DEFINITION */
/***********************************************/
/* Sensor Specific constants */
#define BMP280_SLEEP_MODE (0x00)
#define BMP280_FORCED_MODE (0x01)
#define BMP280_NORMAL_MODE (0x03)
#define BMP280_SOFT_RESET_CODE (0xB6)
/************************************************/
/**\name STANDBY TIME DEFINITION */
/***********************************************/
#define BMP280_STANDBY_TIME_1_MS (0x00 << 5)
#define BMP280_STANDBY_TIME_63_MS (0x01 << 5)
#define BMP280_STANDBY_TIME_125_MS (0x02 << 5)
#define BMP280_STANDBY_TIME_250_MS (0x03 << 5)
#define BMP280_STANDBY_TIME_500_MS (0x04 << 5)
#define BMP280_STANDBY_TIME_1000_MS (0x05 << 5)
#define BMP280_STANDBY_TIME_2000_MS (0x06 << 5)
#define BMP280_STANDBY_TIME_4000_MS (0x07 << 5)
/************************************************/
/**\name OVERSAMPLING TEMPERATURE DEFINITION */
/***********************************************/
#define BMP280_OVERSAMPT_SKIPPED (0x00 << 5)
#define BMP280_OVERSAMPT_1X (0x01 << 5)
#define BMP280_OVERSAMPT_2X (0x02 << 5)
#define BMP280_OVERSAMPT_4X (0x03 << 5)
#define BMP280_OVERSAMPT_8X (0x04 << 5)
#define BMP280_OVERSAMPT_16X (0x05 << 5)
/************************************************/
/**\name OVERSAMPLING PRESSURE DEFINITION */
/***********************************************/
#define BMP280_OVERSAMPP_SKIPPED (0x00 << 2)
#define BMP280_OVERSAMPP_1X (0x01 << 2)
#define BMP280_OVERSAMPP_2X (0x02 << 2)
#define BMP280_OVERSAMPP_4X (0x03 << 2)
#define BMP280_OVERSAMPP_8X (0x04 << 2)
#define BMP280_OVERSAMPP_16X (0x05 << 2)
/************************************************/
/**\name FILTER DEFINITION */
/***********************************************/
#define BMP280_FILTER_COEFF_OFF (0x00 << 2)
#define BMP280_FILTER_COEFF_2 (0x01 << 2)
#define BMP280_FILTER_COEFF_4 (0x02 << 2)
#define BMP280_FILTER_COEFF_8 (0x03 << 2)
#define BMP280_FILTER_COEFF_16 (0x04 << 2)


/** Usage mode are defined below
 *  */
#define BMP280_HANDHELD_LOW_POWER_MODE    0	// 247uA 10Hz
#define BMP280_HANDHELD_DYNAMIC_MODE      1	// 577uA 83Hz
#define BMP280_WEATHER_MONITORING_MODE    2	// 0.14uA 1/60 Hz -> need restart mode for new measure
#define BMP280_ELEVATOR_MODE              3	// 50uA 7.3Hz
#define BMP280_DROP_DETECT_MODE           4	// 509uA 125Hz
#define BMP280_INDOOR_NAVIGATION_MODE     5	// 650uA 26Hz
#define BMP280_SMARTSENSOR_MODE           6   // 0.5uA 1/60Hz  -> need restart mode for new measure


#define BMP280_DEFAULT_MODE			BMP280_SMARTSENSOR_MODE


typedef struct
{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
  int32_t  tFine;
}bmp280_cal_val_t;

/** Polynomial value used in CRC calculation
 * \see check_crc_SHT21 */

/**
 * \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define BMP280_ERR_NONE           0x00
#define BMP280_ERR_GET_CALIB      0x01
#define BMP280_ERR_READ_CALIB     0x02
#define BMP280_ERR_WRITE_MODE     0x03
#define BMP280_ERR_WRITE_REGISTER 0x04
#define BMP280_ERR_READ_ID        0x05
#define BMP280_ERR_BAD_ID         0x06
#define BMP280_ERR_GET_TEMP       0x07
#define BMP280_ERR_READ_TEMP      0x08
#define BMP280_ERR_SET_MODE       0x09
#define BMP280_INVALID_DATA       0x0A
#define BMP280_ERR_READ_STATUS    0x0B
#define BMP280_ERR_POWER_ON       0x0C
#define BMP280_ERR_POWER_OFF      0x0D
#define BMP280_ERR_READ_PRESSURE  0x0E
/** @} */

/**
 * \name User commands
 *
 * @{
 */
enum bmp280_user_command {
  BMP280_TEMP,			/*!< Get temperature */
  BMP280_PRESSURE		/*!< Get pressure */
};
/** @} */



/** User defined states for the configure() and status() function. */
enum {
  BMP280_ACCURACY_MODE,
  BMP280_CURRENT_STATUS,
};


extern const struct sensors_sensor bmp280_sensor;

#endif /* SHT21_SENSOR_H_ */

/**
 * @}
 * @}
 */
