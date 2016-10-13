/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-sht21 SHT21 Driver
 * @{
 *
 * \file
 * Header file for SHT21 sensor driver
 *
 * \author
 * Darko Petrovic
 *
 */


#ifndef SHT21_SENSOR_H_
#define SHT21_SENSOR_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"

#define SHT21_SENSOR          "SHT21"

/** Set the default resolution.
 * <PRE>
 *  0 = RH: 12-bits 	T: 14-bits
 *  1 = RH: 8-bits		T: 12-bits
 *  2 = RH: 10-bits 	T: 13-bits
 *  3 = RH: 11-bits 	T: 11-bits
 *  </PRE>
 *  */
#define SHT21_DEFAULT_RES     0

/**
 * If set, the device goes in PM2 during measurement.
 * In the hold master mode, the SHT2x pulls down the SCL
 * line while measuring to force the master into a wait state.
 * By releasing the SCL line the sensor indicates that internal
 * processing  is  terminated  and  that  transmission  may  be
 * continued.
 */
#define SHT21_NO_HOLD_MODE    1

#define SHT21_PWR_PORT_BASE   GPIO_PORT_TO_BASE(SHT21_PWR_PORT)
#define SHT21_PWR_PIN_MASK    GPIO_PIN_MASK(SHT21_PWR_PIN)

/* -----------------------------------------------------------------*/
/*				/!\ No not alter values below /!\					*/
/* -----------------------------------------------------------------*/

#ifndef SHT21_CONF_DEFT_SENSE
#define SHT21_DEFT_SENSE            SHT21_ALL
#else
#define SHT21_DEFT_SENSE            SHT21_CONF_DEFT_SENSE
#endif


#if SHT21_NO_HOLD_MODE
#define SHT21_MODE            SHT21_NO_HOLD
#else
#define SHT21_MODE            SHT21_HOLD
#endif

/**
 * \name Chip registers
 *
 * @{
 */
#define	SHT21_HOLD                0x00
#define	SHT21_NO_HOLD             0x10
#define	SHT21_CMD_MEAS_T          0xE3
#define SHT21_CMD_MEAS_RH         0xE5
#define SHT21_CMD_USER_WRITE      0xE6	// Write user register
#define SHT21_CMD_USER_READ       0xE7	// Read user register
#define SHT21_CMD_SOFT_RESET      0xFE
/** @} */

/** Polynomial value used in CRC calculation
 * \see check_crc_SHT21 */
#define SHT21_CRC_POLYNOMIAL      0x131

/**
 * \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define SHT21_ERR_NONE            0x00
#define SHT21_ERR_CMD_USER_REG    0x01
#define SHT21_ERR_READ_USER_REG   0x02
#define SHT21_ERR_WRITE_USER_REG  0x03
#define SHT21_ERR_CRC_CHECK       0x04
#define SHT21_ERR_POWER_ON        0x05
#define SHT21_ERR_POWER_OFF       0x06
#define SHT21_ERR_EN_MEASURE      0x10
#define SHT21_ERR_READ_MEASURE    0x20
#define SHT21_ERR_SET_RESOLUTION  0x60
#define SHT21_ERR_SET_HEATER      0x70

/** @} */

/**
 * \name User commands
 * @{
 */
enum sht21_user_command {
  SHT21_TEMP            = 0x01,   /*!< Get temperature */
  SHT21_HUMIDITY        = 0x02,   /*!< Get humidity */
  SHT21_ALL             = 0x03,
};
/** @} */

/**
 * \name User register bits
 *
 * @{
 */
enum sht21_userreg_resolution {
  SHT21_RES_RH12_T14      = 0x00,
  SHT21_RES_RH8_T12       = 0x01,
  SHT21_RES_RH10_T13      = 0x80,
  SHT21_RES_RH11_T11      = 0x81,
  SHT21_RES_MASK          = 0x81
};

enum sht21_userreg_battery {
  SHT21_END_BATTERY       = 0x40, /*!< end of battery */
  SHT21_END_BATTERY_MASK  = 0x40, /*!< Mask for EOB bit(6) in user reg. */
};

enum sht21_userreg_heater {
  SHT21_HEATER_ON          = 0x04, /*!< heater on */
  SHT21_HEATER_OFF         = 0x00, /*!< heater off */
  SHT21_HEATER_MASK        = 0x04, /*!< Mask for Heater bit(2) in user reg. */
};

/** @} */


/** User defined states for the configure() and status() function. */
enum {
  SHT21_RESOLUTION,
  SHT21_HEATER,
  SHT21_OTP
};


extern const struct sensors_sensor sht21_sensor;

#endif /* SHT21_SENSOR_H_ */

/**
 * @}
 * @}
 */
