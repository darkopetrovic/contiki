/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-tmp100 TMP100 Sensor Driver
 *
 * @{
 *
 * \file
 * Header file for the back temperature sensor
 *
 * \author
 * Darko Petrovic
 *
 */

#ifndef TMP100_SENSOR_H_
#define TMP100_SENSOR_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"

#define TMP100_SENSOR 					"TMP100"

/** Set default resolution
 * <PRE>
 * 0 = 9-bits
 * 1 = 10-bits
 * 2 = 11-bits
 * 3 = 12-bits
 * </PRE>
 */
#define TMP100_DEFAULT_RES				3

#define TMP100_PWR_PORT_BASE 	GPIO_PORT_TO_BASE(TMP100_PWR_PORT)
#define TMP100_PWR_PIN_MASK 	GPIO_PIN_MASK(TMP100_PWR_PIN)

/* -----------------------------------------------------------------*/
/*				/!\ No not alter values below /!\					*/
/* -----------------------------------------------------------------*/

/**
 * \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define TMP100_ERR_NONE					0x00
#define TMP100_ERR_READ_CONFIG			0x02
#define TMP100_ERR_WRITE_CONFIG			0x03
#define TMP100_ERR_WRITE_PR				0x04
#define TMP100_ERR_EN_MEASURE			0x10
#define TMP100_ERR_READ_MEASURE			0x20
/** @} */

/**
 * Minimum conversion time for 9-bits of resolution.
 * The other conversion time are based on this value.
 */
#define	TMP100_CONVERSION_TIME_BASE		40	// ms

/**
 * \name  User commands
 * @{
 */
enum {
	TMP100_SENSE_TEMP,		/*!< Get temperature */
};
/** @} */

/**
 * \name  Chip registers
 * @{
 */
enum tmp100_register {
	TMP100_REG_TEMP		= 0x00,
	TMP100_REG_CONFIG	= 0x01,
	TMP100_REG_TLOW		= 0x02,
	TMP100_REG_THIGH	= 0x03
};
/** @} */

/**
 * \name  Config register
 * @{
 */
enum tmp100_config_register {
	TMP100_SHUTDOWN 		= (1 << 0),
	TMP100_THERMOSTAT_MODE	= (1 << 1),
	TMP100_POLARITY 		= (1 << 2),
	TMP100_FAULT_QUEUE_1 	= (0 << 3),
	TMP100_FAULT_QUEUE_2 	= (1 << 3),
	TMP100_FAULT_QUEUE_4 	= (2 << 3),
	TMP100_FAULT_QUEUE_6 	= (3 << 3),
	TMP100_RES_9bits 		= (0 << 5),
	TMP100_RES_10bits		= (1 << 5),
	TMP100_RES_11bits		= (2 << 5),
	TMP100_RES_12bits		= (3 << 5),
	TMP100_RES_MASK			= (3 << 5),
	TMP100_OS_ALERT			= (1 << 7)
};
/** @} */

/** User defined states for the configure() and status() function. */
enum {
	TMP100_RESOLUTION
};

extern const struct sensors_sensor tmp100_sensor;

#endif /* TMP100_SENSOR_H_ */

/**
 * @}
 * @}
 */
