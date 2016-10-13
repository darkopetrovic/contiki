/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-ina3221 Power Sensor Driver
 *
 * @{
 *
 * \file
 * Header file for the power sensor
 *
 * \author
 * Darko Petrovic
 *
 */

#ifndef INA3221_SENSOR_H_
#define INA3221_SENSOR_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"

#define INA3221_SENSOR                  "INA3221"

/* -----------------------------------------------------------------*/
/*				/!\ No not alter values below /!\					*/
/* -----------------------------------------------------------------*/

#define CH1     0x04
#define CH2     0x02
#define CH3     0x01
#define CHALL   0x07

#define VSHUNT   0x10
#define VBUS     0x20
#define VBOTH    0x30

#ifndef INA3221_CONF_DEFT_CH
#define INA3221_DEFT_CH   CHALL
#else
#define INA3221_DEFT_CH   INA3221_CONF_DEFT_CH
#endif

#ifndef INA3221_CONF_DEFT_MODE
#define INA3221_DEFT_MODE   VBOTH
#else
#define INA3221_DEFT_MODE   INA3221_CONF_DEFT_MODE
#endif

/**
 * \name  Chip registers
 * @{
 */
#define INA3221_REG_CONF                0x00
#define INA3221_REG_CHAN1_SVOLT         0x01
#define INA3221_REG_CHAN1_BVOLT         0x02
#define INA3221_REG_CHAN2_SVOLT         0x03
#define INA3221_REG_CHAN2_BVOLT         0x04
#define INA3221_REG_CHAN3_SVOLT         0x05
#define INA3221_REG_CHAN3_BVOLT         0x06
#define INA3221_REG_CHAN1_CRITICAL      0x07
#define INA3221_REG_CHAN1_WARNING       0x08
#define INA3221_REG_CHAN2_CRITICAL      0x09
#define INA3221_REG_CHAN2_WARNING       0x0A
#define INA3221_REG_CHAN3_CRITICAL      0x0B
#define INA3221_REG_CHAN3_WARNING       0x0C
#define INA3221_REG_SVOLT_SUM           0x0D
#define INA3221_REG_SVOLT_SUM_LIMIT     0x0E
#define INA3221_REG_MASK_ENABLE         0x0F
#define INA3221_REG_POWER_VALID_UP      0x10
#define INA3221_REG_POWER_VALID_LO      0x11
#define INA3221_REG_MANUFACTURER_ID     0xFE
#define INA3221_REG_DIE_ID              0xFF
/** @} */

/**
 * \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define INA3221_ERR_NONE                0x00
#define INA3221_ERR_SET_REGISTER        0x01
#define INA3221_ERR_READ_REGISTER       0x02
#define INA3221_ERR_WRITE_REGISTER      0x03
#define INA3221_ERR_EN_MEASURE          0x10
#define INA3221_ERR_READ_MEASURE        0x20

/** @} */
/**
 * \name  User commands
 * @{
 */
enum ina3221_user_command {
  INA3221_CH1_SHUNT_VOLTAGE,			/*!< Get Channel 1 shunt voltage */
  INA3221_CH1_BUS_VOLTAGE,			/*!< Get Channel 1 bus voltage */
  INA3221_CH2_SHUNT_VOLTAGE,			/*!< Get Channel 2 shunt voltage */
  INA3221_CH2_BUS_VOLTAGE,			/*!< Get Channel 2 bus voltage */
  INA3221_CH3_SHUNT_VOLTAGE,			/*!< Get Channel 3 shunt voltage */
  INA3221_CH3_BUS_VOLTAGE				/*!< Get Channel 3 bus voltage */
};
/** @} */

/**
 * \name  Configuration register
 * @{
 */

/** Mode settings (Bits[2:0]) */
enum ina3221_mode_settings {
  INA3221_MODE_POWER_DOWN             = (0 << 0),
  INA3221_MODE_SHUNT_VOLTAGE          = (1 << 0),
  INA3221_MODE_BUS_VOLTAGE            = (2 << 0),
  INA3221_MODE_SHUNT_BUS_VOLTAGE      = (3 << 0),
  INA3221_MODE_POWER_DOWN2            = (4 << 0),
  INA3221_MODE_SHUNT_VOLTAGE_CONT     = (5 << 0),
  INA3221_MODE_BUS_VOLTAGE_CONT       = (6 << 0),
  INA3221_MODE_SHUNT_BUS_VOLTAGE_CONT = (7 << 0) // default
};

/** Conversion time for the SHUNT measurement (Bits[5:3]) */
enum ina3221_convtime_shunt {
  INA3221_CONF_CT_SHUNT_140us         = (0 << 3),
  INA3221_CONF_CT_SHUNT_204us         = (1 << 3),
  INA3221_CONF_CT_SHUNT_332us         = (2 << 3),
  INA3221_CONF_CT_SHUNT_588us         = (3 << 3),
  INA3221_CONF_CT_SHUNT_1100us        = (4 << 3), 	// default
  INA3221_CONF_CT_SHUNT_2116us        = (5 << 3),
  INA3221_CONF_CT_SHUNT_4156us        = (6 << 3),
  INA3221_CONF_CT_SHUNT_8244us        = (7 << 3)
};

/** Conversion time for the BUS measurement (Bits[8:6]) */
enum ina3221_convtime_bus {
  INA3221_CONF_CT_BUS_140us           = (0 << 6),
  INA3221_CONF_CT_BUS_204us           = (1 << 6),
  INA3221_CONF_CT_BUS_332us           = (2 << 6),
  INA3221_CONF_CT_BUS_588us           = (3 << 6),
  INA3221_CONF_CT_BUS_1100us          = (4 << 6), 	// default
  INA3221_CONF_CT_BUS_2116us          = (5 << 6),
  INA3221_CONF_CT_BUS_4156us          = (6 << 6),
  INA3221_CONF_CT_BUS_8244us          = (7 << 6)
};

/** Averaging mode (Bits[11:9]) */
enum ina3221_avg_mode {
  INA3221_CONF_NB_AVG_1               = (0 << 9),			// default
  INA3221_CONF_NB_AVG_4               = (1 << 9),
  INA3221_CONF_NB_AVG_16              = (2 << 9),
  INA3221_CONF_NB_AVG_64              = (3 << 9),
  INA3221_CONF_NB_AVG_128             = (4 << 9),
  INA3221_CONF_NB_AVG_256             = (5 << 9),
  INA3221_CONF_NB_AVG_512             = (6 << 9),
  INA3221_CONF_NB_AVG_1024            = (7 << 9),
};

/** Setting this bit to '1' generates a system reset that is the same as a power-on reset (POR).
 * This bit resets all registers to default values and self-clears. */
#define INA3221_RESET             (1<<15)

/** Enable channel 1 */
#define INA3221_EN_CHANNEL_1      (1<<14)

/** Enable channel 2 */
#define INA3221_EN_CHANNEL_2      (1<<13)

/** Enable channel 3 */
#define INA3221_EN_CHANNEL_3      (1<<12)

/** @} */

/**
 * \name  Mask/Enable Register
 * @{
 */
enum ina3221_masken_reg {
  INA3221_MASKEN_CVRF           = (1 << 0),
  INA3221_MASKEN_TCF            = (1 << 1),
  INA3221_MASKEN_PVF            = (1 << 2),
  INA3221_MASKEN_WF             = (7 << 3),
  INA3221_MASKEN_SF             = (1 << 6),
  INA3221_MASKEN_CF             = (7 << 7),
  INA3221_MASKEN_CEN            = (1 << 10),
  INA3221_MASKEN_WEN            = (1 << 11),
  INA3221_MASKEN_SCC            = (7 << 12),
};

/** @} */

/** Bus voltage LSB */
#define INA3221_BUS_VOLT_LSB    8e-3

/** Shunt voltage LSB */
#define INA3221_SHUNT_VOLT_LSB  40e-6

extern const struct sensors_sensor ina3221_sensor;

#endif /* INA3221_SENSOR_H_ */

/**
 * @}
 * @}
 */
