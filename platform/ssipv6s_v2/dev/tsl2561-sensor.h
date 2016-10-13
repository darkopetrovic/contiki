/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-tsl2561 TSL2561 Driver
 * @{
 *
 * \file
 * Header file for TSL2561 sensor driver
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 *
 */


#ifndef TSL2561_SENSOR_H_
#define TSL2561_SENSOR_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"
#include <stdint.h>

#define TSL2561_SENSOR            "TSL2561"

#define TSL2561_PWR_PORT_BASE     GPIO_PORT_TO_BASE(TSL2561_PWR_PORT)
#define TSL2561_PWR_PIN_MASK      GPIO_PIN_MASK(TSL2561_PWR_PIN)

/* -----------------------------------------------------------------*/
/*				/!\ No not alter values below /!\					*/
/* -----------------------------------------------------------------*/

/**
 * \name Chip registers
 *
 * @{
 */
#define TSL2561_REGISTER_CONTROL            0x00
#define TSL2561_REGISTER_TIMING             0x01
#define TSL2561_REGISTER_THRESHHOLDL_LOW    0x02
#define TSL2561_REGISTER_THRESHHOLDL_HIGH   0x03
#define TSL2561_REGISTER_THRESHHOLDH_LOW    0x04
#define TSL2561_REGISTER_THRESHHOLDH_HIGH   0x05
#define TSL2561_REGISTER_INTERRUPT          0x06
#define TSL2561_REGISTER_CRC                0x08
#define TSL2561_REGISTER_ID                 0x0A
#define TSL2561_REGISTER_CHAN0_LOW          0x0C
#define TSL2561_REGISTER_CHAN0_HIGH         0x0D
#define TSL2561_REGISTER_CHAN1_LOW          0x0E
#define TSL2561_REGISTER_CHAN1_HIGH         0x0F

/** @} */

#define TSL2561_COMMAND_BIT                 0x80      // Must be 1 to give command
#define TSL2561_CLEAR_BIT                   0x40      // Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT                    0x20      // 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT                   0x10      // 1 = using block read/write

// T, FN, and CL Package coefficients
#define TSL2561_LUX_K1T                     0x0040 // 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T                     0x01f2 // 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T                     0x01be // 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T                     0x0080 // 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T                     0x0214 // 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T                     0x02d1 // 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T                     0x00c0 // 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T                     0x023f // 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T                     0x037b // 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T                     0x0100 // 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T                     0x0270 // 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T                     0x03fe // 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T                     0x0138 // 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T                     0x016f // 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T                     0x01fc // 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T                     0x019a // 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T                     0x00d2 // 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T                     0x00fb // 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T                     0x029a // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T                     0x0018 // 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T                     0x0012 // 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T                     0x029a // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T                     0x0000 // 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T                     0x0000 // 0.000 * 2^LUX_SCALE

#define TSL2561_INTEG_13_7MS                0x00
#define TSL2561_INTEG_101MS                 0x01
#define TSL2561_INTEG_402MS                 0x02
#define TSL2561_INTEG_MANUAL                0x03

#define TSL2561_LUX_LUXSCALE                14 // Scale by 2^14
#define TSL2561_LUX_RATIOSCALE              9 // Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE                 10 // Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0           0x7517 // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1           0x0FE7 // 322/81 * 2^TSL2561_LUX_CHSCALE
/** Polynomial value used in CRC calculation
 * \see check_crc_SHT21 */

/**
 * \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define TSL2561_ERR_NONE                0x00
#define TSL2561_ERR_CMD_SEND            0x01
#define TSL2561_ERR_MEASURE_READ        0x02
#define TSL2561_ERR_TIMING_READ         0x03
#define TSL2561_ERR_ID_READ             0x04
#define TSL2561_ERR_BAD_ID              0x05
#define TSL2561_ERR_SET_MODE            0x06
#define TSL2561_ERR_POWER_ON            0x07
#define TSL2561_ERR_POWER_OFF           0x08
#define TSL2561_ERR_ACTIVATE            0x09
#define TSL2561_ERR_LIGHT               0x0A

#define TSL2561_ID_DATA                 0x10	// 1 is TSL2561, 4 lsb are revision
/** @} */

/** Usage mode are defined below
 *  */
#define TSL2561_G1_T13                 0b00001000	// gain x1, integration time 13.7ms
#define TSL2561_G1_T101                0b00001001	// gain x1, integration time 101ms
#define TSL2561_G1_T402                0b00001010	// gain x1, integration time 402ms
#define TSL2561_G1_TMAN                0b00001011	// gain x1, integration time manual driver
#define TSL2561_G16_T13                0b00011000	// gain x16, integration time 13.7ms
#define TSL2561_G16_T101               0b00011001	// gain x16, integration time 101ms
#define TSL2561_G16_T402               0b00011010	// gain x16, integration time 402ms
#define TSL2561_G16_TMAN               0b00011011	// gain x16, integration time manual driver


/**
 * \name User commands
 *
 * @{
 */
enum tsl2561_user_command {
  TSL2561_LUX,  /*!< Get lux */
  TSL2561_IR,
  TSL2561_VISIBLE
};
/** @} */



/** User defined states for the configure() and status() function. */
enum {
  TSL2561_MODE,
  TSL2561_CUR_MODE
};


extern const struct sensors_sensor tsl2561_sensor;

#endif /* TSL2561_SENSOR_H_ */

/**
 * @}
 * @}
 */
