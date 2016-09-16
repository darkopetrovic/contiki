/**
 * \addtogroup platform
 * @{
 *
 * \defgroup sensors Sensors
 *
 * @{
 *
 * \file
 * Implementation of a generic module controlling the platform sensors
 */

#ifndef PLATFORM_SENSOR_H_
#define PLATFORM_SENSOR_H_


void deep_sleep_ms(uint32_t duration);
uint16_t get_battery_voltage(void);

#endif /* PLATFORM_SENSOR_H_ */

/**
 * @}
 * @}
 */
