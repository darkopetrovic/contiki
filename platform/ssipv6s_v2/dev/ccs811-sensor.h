/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-ccs811 CCS811 Driver
 * @{
 *
 * \file
 * Header file for CCS811 sensor driver
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 *
 */


#ifndef CCS811_SENSOR_H_
#define CCS811_SENSOR_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"
#include <stdint.h>

#define CCS811_SENSOR         "CCS811"

/**
 * If set, the device goes in PM2 during measurement.
 * In the hold master mode, the SHT2x pulls down the SCL
 * line while measuring to force the master into a wait state.
 * By releasing the SCL line the sensor indicates that internal
 * processing  is  terminated  and  that  transmission  may  be
 * continued.
 */
#define CCS811_NO_HOLD_MODE     1

#define CCS811_PWR_PORT_BASE    GPIO_PORT_TO_BASE(CCS811_PWR_PORT)
#define CCS811_PWR_PIN_MASK     GPIO_PIN_MASK(CCS811_PWR_PIN)
#define CCS811_NWAKE_PORT_BASE  GPIO_PORT_TO_BASE(CCS811_NWAKE_PORT)
#define CCS811_NWAKE_PIN_MASK   GPIO_PIN_MASK(CCS811_NWAKE_PIN)
#define CCS811_RST_PORT_BASE    GPIO_PORT_TO_BASE(CCS811_RST_PORT)
#define CCS811_RST_PIN_MASK     GPIO_PIN_MASK(CCS811_RST_PIN)

/* -----------------------------------------------------------------*/
/*        /!\ No not alter values below /!\         */
/* -----------------------------------------------------------------*/

#if CCS811_NO_HOLD_MODE
#define CCS811_MODE   CCS811_NO_HOLD
#else
#define CCS811_MODE   CCS811_HOLD
#endif

/**
 * \name Chip registers
 *
 * @{
 */

#define CCS811_STATUS           0x00
#define CCS811_MEAS_MODE        0x01
#define CCS811_ALG_RESULT_DATA  0x02
#define CCS811_RAW_DATA         0x03
#define CCS811_ENV_DATA         0x05
#define CCS811_NTC              0x06
#define CCS811_THRESHOLDS       0x10
#define CCS811_BASELINE         0x11
#define CCS811_HW_ID            0x20
#define CCS811_HW_ID_DATA       0x81
#define CCS811_HW VERSION       0x21
#define CCS811_FW_BOOT_VERSION  0x23
#define CCS811_FW_APP_VERSION   0x24
#define CCS811_ERROR_ID         0xE0
#define CCS811_SW_RESET         0xFF

#define CCS811_APP_START        0xF4

/** @} */
/****************************************************/
/**\name CCS811_STATUS DEFINITIONS          */
/****************************************************/
#define CCS811_FW_MODE        (1 << 7)
#define CCS811_APP_VALID      (1 << 4)
#define CCS811_DATA_RDY       (1 << 3)
#define CCS811_ERROR          (1 << 0)
/****************************************************/
/**\name CCS811_MEAS_MODE DEFINITION        */
/****************************************************/
#define CCS811_DRIVE_MODE_0     (0 << 4)
#define CCS811_DRIVE_MODE_1     (1 << 4)
#define CCS811_DRIVE_MODE_2     (2 << 4)
#define CCS811_DRIVE_MODE_3     (3 << 4)
#define CCS811_DRIVE_MODE_4     (4 << 4)
#define CCS811_INTERRUPT        (1 << 3)
#define CCS811_THRES            (1 << 2)
/****************************************************/
/**\name CCS811_ERROR_ID DEFINITION         */
/****************************************************/
#define CCS_811_HEATER_SUPPLY     (1 << 5)
#define CCS_811_HEATER_FAULT      (1 << 4)
#define CCS_811_MAX_RESISTANCE    (1 << 3)
#define CCS_811_MEASMODE_INVALID  (1 << 2)
#define CCS_811_READ_REG_INVALID  (1 << 1)
#define CCS_811_MSG_INVALID       (1 << 0)


#define CCS811_MODE_IDLE        CCS811_DRIVE_MODE_0
#define CCS811_MODE_SECOND      CCS811_DRIVE_MODE_1
#define CCS811_MODE_10_SECONDS  CCS811_DRIVE_MODE_2
#define CCS811_MODE_60_SECONDS  CCS811_DRIVE_MODE_3
#define CCS811_MODE_250_MS      CCS811_DRIVE_MODE_4





 /* \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define  CCS811_ERR_NONE              0x00
#define  CCS811_ERR_WRITE_REGISTER    0x01
#define  CCS811_ERR_READ_ID           0x02
#define  CCS811_ERR_BAD_ID            0x03
#define  CCS811_ERR_WRITE_MODE        0x04
#define  CCS811_ERR_READ_STATUS       0x05
#define  CCS811_ERR_READ_MODE         0x06
#define  CCS811_ERR_POWER_OFF         0x07
#define  CCS811_ERR_POWER_ON          0x08
#define  CCS811_ERR_SET_MODE          0x09
#define  CCS811_ERR_READ_MEASURE      0x0A
#define  CCS811_ERR_READ_BASELINE     0x0B
#define  CCS811_ERR_SET_BASELINE      0x0C
#define  CCS811_ERR_WRITE_ENV_DATA    0x0D
#define  CCS811_ERR_APP_START         0x0E
/** @} */

/**
 * \name User commands
 *
 * @{
 */
enum ccs811_user_command {
  CCS811_SENSE_CO2,     /*!< Get CO2 im ppm */
  CCS811_SENSE_TVOC     /*!< Get Total Volative Organic Compound in ppb */
};
/** @} */



/** User defined states for the configure() and status() function. */
enum ccs811_functions {
  CCS811_POWER_STATE,
  CCS811_CURRENT_STATUS,
  CCS811_CURRENT_MODE,
  CCS811_CURRENT_BASELINE,
  CCS811_CURRENT_TEMPERATURE,
  CCS811_CURRENT_HUMIDITY,

};


extern const struct sensors_sensor ccs811_sensor;

#endif /* CCS811_SENSOR_H_ */

/**
 * @}
 * @}
 */
