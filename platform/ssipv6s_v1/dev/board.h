/** \addtogroup platform
 * @{
 *
 * \defgroup peripherals Peripherals
 *
 * Defines related to the SSIPv6S platform
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other SSIPv6S peripherals
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

/** Pin PA_7 (RF2.12) activates the boot loader */
#undef FLASH_CCA_CONF_BOOTLDR_BACKDOOR_PORT_A_PIN
#define FLASH_CCA_CONF_BOOTLDR_BACKDOOR_PORT_A_PIN    7

/*---------------------------------------------------------------------------*/
/** \name SmartSensor LED configuration
 *
 * LEDs on the smart sensor are connected as follows:
 * - LED1 (Red)    -> PC1
 * - LED2 (Yellow) -> PC2
 *
 * @{
 */
/*---------------------------------------------------------------------------*/
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED
#undef LEDS_CONF_ALL

#define LEDS_RED          2 /**< LED2 (Red) -> PC1 */
#define LEDS_YELLOW       4 /**< LED1 (Yellow)  -> PC2 */
#define LEDS_CONF_ALL     6

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1

/** @} */
/*---------------------------------------------------------------------------*/
/** \name USB configuration
 *
 * The USB pullup is driven by PC0
 */
#define USB_PULLUP_PORT           GPIO_C_NUM
#define USB_PULLUP_PIN            0

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
#define UART0_RX_PORT            GPIO_A_NUM
#define UART0_RX_PIN             0

#define UART0_TX_PORT            GPIO_A_NUM
#define UART0_TX_PIN             1

#define UART1_CTS_PORT           GPIO_B_NUM
#define UART1_CTS_PIN            5

#define UART1_RTS_PORT           GPIO_A_NUM
#define UART1_RTS_PIN            6
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SmartSensor Button configuration
 *
 * Buttons on the SmartSensor are connected as follows:
 * - BUTTON_SELECT -> PB1 (this button can be used when the board is plugged on the SmartRF board)
 * - BUTTON_USER -> PD0

 * @{
 */
/** BUTTON_SELECT -> PB1 */
#define BUTTON_SELECT_PORT      GPIO_B_NUM
#define BUTTON_SELECT_PIN       1
#define BUTTON_SELECT_VECTOR    NVIC_INT_GPIO_PORT_B

/** BUTTON_USER -> PD0 */
#define BUTTON_USER_PORT        GPIO_D_NUM
#define BUTTON_USER_PIN         0
#define BUTTON_USER_VECTOR      NVIC_INT_GPIO_PORT_D

/* USB Plug detection */
#define	USB_PLUG_DETECT_PORT    GPIO_C_NUM
#define	USB_PLUG_DETECT_PIN     4
#define USB_PLUG_DETECT_VECTOR  NVIC_INT_GPIO_PORT_C

#define USB_IS_PLUGGED()        GPIO_READ_PIN(GPIO_PORT_TO_BASE(USB_PLUG_DETECT_PORT), \
                                GPIO_PIN_MASK(USB_PLUG_DETECT_PIN))

/* Notify various examples that we have Buttons */
#define PLATFORM_HAS_BUTTON      1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ADC configuration
 *
 * These values configure which CC2538 pins and ADC channels to use for the ADC
 * inputs.
 *
 * ADC inputs can only be on port A.
 * @{
 */

//#define ADC_ALS_PWR_PORT         GPIO_A_NUM /**< ALS power GPIO control port */
//#define ADC_ALS_PWR_PIN          7 /**< ALS power GPIO control pin */
//#define ADC_ALS_OUT_PIN          6 /**< ALS output ADC input pin on port A */

/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \name I2C pins
 * @{
 */
#define I2C_SDA_PORT              GPIO_A_NUM
#define I2C_SDA_PIN               4
#define I2C_SCL_PORT              GPIO_A_NUM
#define I2C_SCL_PIN               5
/** @} */
/*---------------------------------------------------------------------------*/

/**
 * \name SPI configuration
 *
 * These values configure which CC2538 pins to use for the SPI lines.
 * @{
 */
#define SPI_CLK_PORT             GPIO_B_NUM
#define SPI_CLK_PIN              0
#define SPI_MOSI_PORT            GPIO_C_NUM
#define SPI_MOSI_PIN             7
#define SPI_MISO_PORT            GPIO_C_NUM
#define SPI_MISO_PIN             5
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name SHT21 sensor configuration (humidity and temperature)
 *
 * @{
 */
#define SHT21_PWR_PORT          GPIO_D_NUM
#define SHT21_PWR_PIN           2
#define SHT21_SLAVE_ADDRESS     0x40  /* S|1|0|0|0|0|0|0|RW */

/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name TMP100 sensor configuration (temperature)
 *
 * @{
 */
#define TMP100_PWR_PORT         GPIO_D_NUM
#define TMP100_PWR_PIN          3
#define TMP100_SLAVE_ADDRESS    0x48
/** @} */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \name PIR sensor configuration (movement detector)
 *
 * @{
 */
#define PIR_INPUT_PORT          GPIO_D_NUM
#define PIR_INPUT_PIN           1
#define PIR_INPUT_VECTOR        NVIC_INT_GPIO_PORT_D
#define PIR_ENABLE_PORT         GPIO_D_NUM
#define PIR_ENABLE_PIN          4

/** @} */
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/**
 * \name INA3221 sensor configuration
 *
 * @{
 */
#define INA3221_SLAVE_ADDRESS         0x41  /* S|1|0|0|0|0|0|1|RW */
#define INA3221_SHUNT_RESISTOR_CH1    10		// Ohm
#define INA3221_SHUNT_RESISTOR_CH2    0.15	// Ohm


/** @} */
/*---------------------------------------------------------------------------*/

/**
 * \name Battery related parameters
 *
 * @{
 */
#define PLATFORM_HAS_BATTERY          1
#if PLATFORM_HAS_BATTERY
#define BATTERY_NOM_VOLTAGE           3.1		// V
#define BATTERY_CUT_VOLTAGE           2.5		// V
#define BATTERY_CAPACITY              0.050	// Ah
#define	BATTERY_SAFE_VOLTAGE          2600	// mV
#define	BATTERY_CRITICAL_VOLTAGE      2480	// mV
#endif
/** @} */


/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "SmartSensor v1.2"
/** @} */

#endif /* BOARD_H_ */

/**
 * @}
 * @}
 */
