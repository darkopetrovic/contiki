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

#define NO_GPIO_INTERRUPT           -1

#define INA3221_CONF_DEFT_CH        CH1 | CH2
#define INA3221_CONF_DEFT_MODE      VBOTH

#define USB_REG_EN_PORT_BASE        GPIO_PORT_TO_BASE(USB_REG_EN_PORT)
#define USB_REG_EN_PIN_MASK         GPIO_PIN_MASK(USB_REG_EN_PIN)

#define USB_REG_ENABLE()            GPIO_CLR_PIN(USB_REG_EN_PORT_BASE, USB_REG_EN_PIN_MASK);\
                                    ioc_set_over(USB_REG_EN_PORT, USB_REG_EN_PIN, IOC_OVERRIDE_PDE)

#define USB_REG_DISABLE()           GPIO_SET_PIN(USB_REG_EN_PORT_BASE, USB_REG_EN_PIN_MASK);\
                                    ioc_set_over(USB_REG_EN_PORT, USB_REG_EN_PIN, IOC_OVERRIDE_PUE)

#define USB_REG_IS_EN()             !GPIO_READ_PIN(USB_REG_EN_PORT_BASE, USB_REG_EN_PIN_MASK)

void deep_sleep_ms(uint32_t duration, int8_t port, uint8_t interrupt_pin);
uint16_t get_battery_voltage(void);
extern uint8_t reading_voltage;

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
uint8_t usb_shell_init(void);
#endif

#endif /* PLATFORM_SENSOR_H_ */

/**
 * @}
 * @}
 */
