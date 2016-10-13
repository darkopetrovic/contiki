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

#define NO_GPIO_INTERRUPT   -1

#define INA3221_CONF_DEFT_CH    CH1 | CH2
#define INA3221_CONF_DEFT_MODE  VBOTH

void deep_sleep_ms(uint32_t duration, int8_t port, uint8_t interrupt_pin);
uint16_t get_battery_voltage(void);

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
uint8_t usb_shell_init(void);
#endif

#endif /* PLATFORM_SENSOR_H_ */

/**
 * @}
 * @}
 */
