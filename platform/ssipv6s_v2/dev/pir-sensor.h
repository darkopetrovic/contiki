/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-pir PIR Sensor Driver
 *
 * @{
 *
 * \file
 * Header file for the PIR sensor
 *
 * \author
 * Darko Petrovic
 *
 */

#ifndef PIR_SENSOR_H_
#define PIR_SENSOR_H_

#include "lib/sensors.h"
#include "dev/ioc.h"
#include "platform-sensors.h"

#define PIR_SENSOR              "PIR"

/* -----------------------------------------------------------------*/
/*				/!\ Do not alter values below /!\					*/
/* -----------------------------------------------------------------*/

#define PIR_INPUT_PORT_BASE     GPIO_PORT_TO_BASE(PIR_INPUT_PORT)
#define PIR_INPUT_PIN_MASK      GPIO_PIN_MASK(PIR_INPUT_PIN)

#define PIR_ENABLE_PORT_BASE    GPIO_PORT_TO_BASE(PIR_ENABLE_PORT)
#define PIR_ENABLE_PIN_MASK     GPIO_PIN_MASK(PIR_ENABLE_PIN)

/** Adjust the number of seconds according the startup sequence of the PIR detector. */
#define PIR_STARTUP_TIME        6 // in seconds

/** Power-ups the PIR sensor. The function starts a timeout to enable the interrupt after
 * the pir sensor has finished to power-up. */
#define	PIR_CIRCUIT_ON()      ioc_set_over(PIR_ENABLE_PORT, PIR_ENABLE_PIN, IOC_OVERRIDE_PDE);\
                              internal_status |= PIR_FLAG_POWERED;\
                              ctimer_set(&pir_timer, CLOCK_SECOND * PIR_STARTUP_TIME, enable_interrupt, NULL)

/** Power-down the PIR sensor. */
#define	PIR_CIRCUIT_OFF()     ioc_set_over(PIR_ENABLE_PORT, PIR_ENABLE_PIN, IOC_OVERRIDE_PUE);\
                              internal_status &= ~(PIR_FLAG_POWERED)

/** Enable PIR sensor interrupt.*/
#define PIR_IRQ_ENABLE()    GPIO_ENABLE_INTERRUPT(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);\
                            GPIO_ENABLE_POWER_UP_INTERRUPT(PIR_INPUT_PORT, PIR_INPUT_PIN_MASK);\
                            internal_status |= PIR_FLAG_ACTIVATED

/** Disable PIR sensor interrupt for the processor. */
#define PIR_IRQ_DISABLE()   GPIO_DISABLE_INTERRUPT(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);\
                            GPIO_DISABLE_POWER_UP_INTERRUPT(PIR_INPUT_PORT, PIR_INPUT_PIN_MASK);\
                            internal_status &= ~(PIR_FLAG_ACTIVATED)


#define PIR_FLAG_POWERED          (1<<0)
#define PIR_FLAG_ACTIVATED        (1<<1)
#define PIR_FLAG_REMAIN_POWERED   (1<<2)

/**
 *  User defined states for the \ref configure() and \ref status() function.
 */
enum {
  PIR_POWER,				/*!< Power state of the sensor */
  PIR_ACTIVATED,			/*!< State of the sensor */
  PIR_DEACTIVATION_DELAY	/*!< Configuration of the deactivation delay */
};

extern const struct sensors_sensor pir_sensor;

/*!************************************************************************************
 * \brief 		Initilization function for the pir sensor.
 * 				Set to 0 the internal status of the sensor and the deactivation delay.
 ************************************************************************************/
void pir_sensor_init();

/*!************************************************************************************
 * \brief 			Deactivate the PIR detection for some seconds.
 *					Starts a debounce timer for the GPIO interrupt.
 *
 * \param seconds 	Number of seconds to deactivate the sensor
 ************************************************************************************/
void pir_irq_delay(uint32_t seconds);

/*!************************************************************************************
 * \brief 			Disable the detection for a certain amount of time.
 * 					The PIR sensor circuitry is completely powered down during
 * 					this time.
 *
 * \param seconds 	Number of seconds to deactivate the sensor
 ************************************************************************************/
void pir_delay_detection(uint32_t seconds);

/*!************************************************************************************
 * \brief 		Get the timer responsible to wake-up the pir sensor after
 * 				the deactivation.
 * 				The function is used by the custom-rdc program to create the
 * 				wake-up based on this timer.
 *
 * \return 		 timer responsible to wake-up the pir sensor
 ************************************************************************************/
struct timer* pir_get_timer();

/*---------------------------------------------------------------------------*/
#endif /* PIR_SENSOR_H_ */


/**
 * @}
 * @}
 */
