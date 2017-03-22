/** \addtogroup platform
 * @{
 *
 * \defgroup peripherals Peripherals
 *
 * Defines related to the SSIPv6S version 2.0 platform
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other SSIPv6S version 2.0 peripherals
 *
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * CC2538 chip
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
#ifndef BOARD_H_
#define BOARD_H_

#include "dev/gpio.h"
#include "dev/nvic.h"

#undef CONTIKI_TARGET_CC2538DK
#define CONTIKI_TARGET_CC2538DK                        1

#undef FLASH_CCA_CONF_BOOTLDR_BACKDOOR_PORT_A_PIN
#define FLASH_CCA_CONF_BOOTLDR_BACKDOOR_PORT_A_PIN     2

/*---------------------------------------------------------------------------*/
/** \name SmartSensor LED configuration
 *
 * LEDs on the smart sensor are connected as follows:
 * - LED1 (Red)    -> PD2
 * - LED2 (Yellow) -> PD1
 *
 * @{
 */
/*---------------------------------------------------------------------------*/
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED
#undef LEDS_BLUE
#undef LEDS_CONF_ALL

#define LEDS_RED          4 /**< LED2 (Red) -> PD2 */
#define LEDS_YELLOW       2 /**< LED1 (Yellow)  -> PD1 */
#define LEDS_CONF_ALL     6

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1

/** @} */
/*---------------------------------------------------------------------------*/
/** \name USB configuration
 *
 * The USB pullup is driven by PC0
 */
#define USB_PULLUP_PORT       GPIO_C_NUM
#define USB_PULLUP_PIN        0
#define USB_REG_EN_PORT       GPIO_C_NUM
#define USB_REG_EN_PIN        7

/** @} */
/*---------------------------------------------------------------------------*/
/** \name UART configuration
 *
 * On the SmartRF06EB, the UART (XDS back channel) is connected to the
 * following ports/pins
 * - RX:  PA0
 * - TX:  PA1
 * - CTS: PB0 (Can only be used with UART1)
 * - RTS: PD3 (Can only be used with UART1)
 *
 * We configure the port to use UART0. To use UART1, replace UART0_* with
 * UART1_* below.
 * @{
 */
#define UART0_RX_PORT            GPIO_B_NUM
#define UART0_RX_PIN             4

#define UART0_TX_PORT            GPIO_D_NUM
#define UART0_TX_PIN             4

/*#define UART1_CTS_PORT           GPIO_B_NUM
#define UART1_CTS_PIN            5

#define UART1_RTS_PORT           GPIO_A_NUM
#define UART1_RTS_PIN            6*/
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SmartSensor Button configuration
 *
 * Buttons on the SmartSensor 2.0 are connected as follows:
 * - BUTTON_USER -> PD0
 * @{
 */
/** BUTTON_USER -> PD0 */
#define BUTTON_USER_PORT        GPIO_D_NUM
#define BUTTON_USER_PIN         0
#define BUTTON_USER_VECTOR      NVIC_INT_GPIO_PORT_D

#define USER_BTN_PRESSED()    !GPIO_READ_PIN(GPIO_PORT_TO_BASE(BUTTON_USER_PORT), \
                              GPIO_PIN_MASK(BUTTON_USER_PIN))

/* USB Plug detection */
#define	USB_PLUG_DETECT_PORT    GPIO_D_NUM
#define	USB_PLUG_DETECT_PIN     5
#define USB_PLUG_DETECT_VECTOR  NVIC_INT_GPIO_PORT_D

#define USB_IS_PLUGGED()    GPIO_READ_PIN(GPIO_PORT_TO_BASE(USB_PLUG_DETECT_PORT), \
                            GPIO_PIN_MASK(USB_PLUG_DETECT_PIN))

/* Notify various examples that we have Buttons */
#define PLATFORM_HAS_BUTTON      1
/** @} */

/*---------------------------------------------------------------------------*/
/** \name I2C pins
 * @{
 */
#define I2C_SDA_PORT               GPIO_A_NUM
#define I2C_SDA_PIN                7
#define I2C_SCL_PORT               GPIO_A_NUM
#define I2C_SCL_PIN                6
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name SHT21 sensor configuration (humidity and temperature)
 *
 * @{
 */
#define SHT21_PWR_PORT            GPIO_C_NUM
#define SHT21_PWR_PIN             5
#define SHT21_SLAVE_ADDRESS       0x40  /* S|1|0|0|0|0|0|0|RW */
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name CCS811 sensor configuration (IAQ)
 *
 * @{
 */
#define CCS811_RST_PORT           GPIO_D_NUM
#define CCS811_RST_PIN            4
#define CCS811_PWR_PORT           GPIO_C_NUM
#define CCS811_PWR_PIN            6
#define CCS811_NWAKE_PORT         GPIO_B_NUM
#define CCS811_NWAKE_PIN          3
#define CCS811_SLAVE_ADDRESS      0x5A
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name BMP280 sensor configuration (pressure)
 *
 * @{
 */
#define BMP280_PWR_PORT           GPIO_C_NUM
#define BMP280_PWR_PIN            4
#define BMP280_SLAVE_ADDRESS      0x77
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name TSL2561 sensor configuration (light)
 *
 * @{
 */
#define TSL2561_PWR_PORT          GPIO_C_NUM
#define TSL2561_PWR_PIN           1
#define TSL2561_SLAVE_ADDRESS     0x29
#define TSL2561_INT_PORT          GPIO_B_NUM
#define TSL2561_INT_PIN           0
#define TSL_INPUT_VECTOR          NVIC_INT_GPIO_PORT_B
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name PCA9546 multiplexer configuration (choose light, pressure, temp, hum.)
 *
 * @{
 */
#define PCA9546_RST_PORT          GPIO_D_NUM
#define PCA9546_RST_PIN           3
#define PCA9546_SLAVE_ADDRESS     0x70
#define PCA_9546_SHT21_SEL_POS    0
#define PCA_9546_BMP280_SEL_POS   1
#define PCA_9546_CCS811_SEL_POS   2
#define PCA_9546_TSL2561_SEL_POS  3
/** @} */

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name INA3221 sensor configuration
 *
 * @{
 */
#define INA3221_SLAVE_ADDRESS         0x41  /* S|1|0|0|0|0|0|1|RW */
#define INA3221_SHUNT_RESISTOR_CH1    10    // Ohm
#define INA3221_SHUNT_RESISTOR_CH2    0.15  // Ohm
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name PIR sensor configuration (movement detector)
 *
 * @{
 */
#define PIR_INPUT_PORT            GPIO_A_NUM
#define PIR_INPUT_PIN             1
#define PIR_INPUT_VECTOR          NVIC_INT_GPIO_PORT_A
#define PIR_PWR_PORT              GPIO_C_NUM
#define PIR_PWR_PIN               3

/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name ICS-40310 microphone configuration
 *
 * @{
 */
#define MIC_PWR_PORT              GPIO_C_NUM
#define MIC_PWR_PIN               2
#define MIC_ADC_INPUT_PORT        GPIO_A_NUM
#define MIC_ADC_INPUT_PIN         0
#define MIC_COMP_INPUT_PORT       GPIO_A_NUM
#define MIC_COMP_INPUT_PIN        5
#define MIC_COMP_INPUT_VECTOR     NVIC_INT_GPIO_PORT_A
#define MIC_COMP_REF_PORT         GPIO_A_NUM
#define MIC_COMP_REF_PIN          4
/** @} */
/*---------------------------------------------------------------------------*/

/**
 * \name Battery related parameters
 *
 * @{
 */
#define PLATFORM_HAS_BATTERY        1
#if PLATFORM_HAS_BATTERY
#define BATTERY_NOM_VOLTAGE         3.1		// V
#define BATTERY_CUT_VOLTAGE         2.5		// V
#define BATTERY_CAPACITY            0.050	// Ah
#define	BATTERY_SAFE_VOLTAGE        2600	// mV
#define	BATTERY_CRITICAL_VOLTAGE    2480	// mV
#endif
/** @} */


/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING                "SmartSensor"
/** @} */

#endif /* BOARD_H_ */

/**
 * @}
 * @}
 */
