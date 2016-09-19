/**
 * \addtogroup sensors
 * @{
 *
 * \file
 * Driver for the PIR sensor
 *
 * \author
 * Darko Petrovic
 */

#include "pir-sensor.h"

/** \cond */
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/** \endcond */

static struct timer pirdebouncetimer;
static struct ctimer pir_timer;
static uint8_t internal_status;
static uint32_t pir_delay_duration;		// in minutes

/*!**********************************************************************************
 * \brief 		Callback for the ctimer used to enable the interrupt after power-up.
 * 				The pir sensor requires some seconds to power-up. The time required
 * 				by the pir sensor to wake-ups is configured by \ref PIR_STARTUP_TIME.
 ***********************************************************************************/
static void
enable_interrupt(void *ptr)
{
	pir_irq_delay(CLOCK_SECOND*3);
	PIR_IRQ_ENABLE();
	PRINTF("Sensor [PIR]: Activation in 3 seconds.\n");
}


/*!**********************************************************************************
 * \brief 		Callback registered with the GPIO module. Gets fired with a button
 * 				port/pin generates an interrupt
 *
 * \param port 	The port number that generated the interrupt
 * \param pin 	The pin number that generated the interrupt. This is the pin
 * 				absolute number (i.e. 0, 1, ..., 7), not a mask
 ***********************************************************************************/
static void
pir_callback(uint8_t port, uint8_t pin)
{

	// Block the motion detection for some seconds.
	// The delay is started the first time with the function below.
	if(!timer_expired(&pirdebouncetimer)) {
		return;
	}
	pir_irq_delay(CLOCK_SECOND*3);

	if(port == GPIO_D_NUM) {
		PRINTF("Sensor [PIR]: Motion detected!\n");
		sensors_changed(&pir_sensor);
		pir_delay_detection(pir_delay_duration);
	}
}

/*!************************************************************************************
 * \brief 		Configure function provided by the sensors API
 *
 * \param type 	Configuration type.
 * \param value Used as binary value to change the state of the sensor
 * 				or as a numerical value to configure a specific parameter.
 * \return 		always 0
 ************************************************************************************/
static int
configure(int type, int value)
{
	switch(type) {
		case SENSORS_HW_INIT:
		  /* Software controlled */
		  GPIO_SOFTWARE_CONTROL(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);

		  /* Set pin to input */
		  GPIO_SET_INPUT(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);

		  /* Enable edge detection */
		  GPIO_DETECT_EDGE(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);

		  /* Single edge */
		  GPIO_TRIGGER_SINGLE_EDGE(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);

		  /* Trigger interrupt on rising edge */
		  GPIO_DETECT_RISING(PIR_INPUT_PORT_BASE, PIR_INPUT_PIN_MASK);

		  /* Power-up the SoC from sleep if detection occurs */
		  GPIO_POWER_UP_ON_FALLING(PIR_INPUT_PORT, PIR_INPUT_PIN_MASK);

		  /* Do not activate immediatly the interrupt */
		  GPIO_DISABLE_POWER_UP_INTERRUPT(PIR_INPUT_PORT, PIR_INPUT_PIN_MASK);

		  /* Disable the default pull-up on the input pin. */
		  ioc_set_over(PIR_INPUT_PORT, PIR_INPUT_PIN, IOC_OVERRIDE_DIS);

		  /* The USER button is on the same port and enable already the interrupt.
		   * We do not need to enable it again here but for the clarity. */
		  nvic_interrupt_enable(PIR_INPUT_VECTOR);

		  /* Callback function when the gpio pin is triggered. */
		  gpio_register_callback(pir_callback, PIR_INPUT_PORT, PIR_INPUT_PIN);

		  pir_sensor_init();

		  /* By default turn motion detection off (default driver behavior,
		   * the detection can be activated by default in the upper layer).
		   * If we set the pin as output in high state, a significant current
		   * is drawn from the pin (~140uA).
		   * Thus, in order to set a high state without consumming current,
		   * we set the pin as input with the pull-up resistor. This is sufficient
		   * to turn off the PMOS transistor.
		   */
		  //GPIO_SET_OUTPUT( PIR_ENABLE_PORT_BASE, PIR_ENABLE_PIN_MASK);	// set pin as output high (bad thing)
		  //GPIO_SET_PIN( PIR_ENABLE_PORT_BASE, PIR_ENABLE_PIN_MASK);
		  GPIO_SET_INPUT( PIR_ENABLE_PORT_BASE, PIR_ENABLE_PIN_MASK);		// this this the default state of the pin
		  ioc_set_over(PIR_ENABLE_PORT, PIR_ENABLE_PIN, IOC_OVERRIDE_PUE);	// after a reset, but we set it here anyway

			break;

		case SENSORS_ACTIVE:

			if( value ){
				// power-up the sensor if it isn't
				if( !(internal_status & PIR_FLAG_POWERED) ){
					PIR_CIRCUIT_ON();
					ENERGEST_ON(ENERGEST_TYPE_SENSORS_PIR);
					PRINTF("Sensor [PIR]: Powering-on.\n");
				} else {
					PIR_IRQ_ENABLE();
					PRINTF("Sensor [PIR]: Activation (enable interrupt).\n");
				}
			} else {
				PIR_IRQ_DISABLE();
				PRINTF("Sensor [PIR]: Deactivation (disable interrupt).\n");
				if( !(internal_status & PIR_FLAG_REMAIN_POWERED) ) {
					PIR_CIRCUIT_OFF();
					ENERGEST_OFF(ENERGEST_TYPE_SENSORS_PIR);
					PRINTF("Sensor [PIR]: Power-off.\n");
				}
			}

			break;

		case PIR_POWER:
			if( value ) {
				/* Turn the power ON for the motion detection circuit.
				 * Use this carefully because the startup sequence of the PIR detector
				 * last some seconds and thus consumes a lot of current. */
				PIR_CIRCUIT_ON();
				ENERGEST_ON(ENERGEST_TYPE_SENSORS_PIR);
				PRINTF("Sensor [PIR]: Powering-on.\n");
			} else {
				/* Turn the power OFF for the motion detection circuit. */
				PIR_CIRCUIT_OFF();
				ENERGEST_OFF(ENERGEST_TYPE_SENSORS_PIR);
				PRINTF("Sensor [PIR]: Power-off.\n");
			}
			break;

		case PIR_DEACTIVATION_DELAY:
			pir_delay_duration = (uint32_t)value;
			break;

		default:
			return 1;
	}

  return 0;
}

/*!************************************************************************************
 * \brief 		Status function provided by the sensors API
 *
 * \param type 	Type of status to return
 * \return 		Value of the status
 ************************************************************************************/
static int
status(int type)
{
	switch(type){
		case PIR_POWER:
			if( internal_status & PIR_FLAG_POWERED ){
				return 1;
			} else {
				return 0;
			}
			break;

		case PIR_ACTIVATED:
			if( internal_status & PIR_FLAG_ACTIVATED ){
				return 1;
			} else {
				return 0;
			}
			break;

		case PIR_DEACTIVATION_DELAY:
			return pir_delay_duration;
			break;

		default:
			return -1;
	}
}

/*!************************************************************************************
 * \brief 		Callback function for the ctimer to reactivate the pir sensor after
 * 				some seconds.
 *
 ************************************************************************************/
static void
reactivate_pir(void *ptr)
{
	SENSORS_ACTIVATE(pir_sensor);
}

void
pir_sensor_init()
{
	timer_set(&pirdebouncetimer, 0);
	pir_delay_duration = 0;
	internal_status = 0;
}

void
pir_irq_delay(uint32_t delay)
{
	timer_set(&pirdebouncetimer, delay);
}



void
pir_delay_detection(uint32_t seconds)
{
	if ( seconds != 0 ){
		SENSORS_DEACTIVATE(pir_sensor);
		ctimer_set(&pir_timer, CLOCK_SECOND * seconds, reactivate_pir, NULL);
		PRINTF("Sensor [PIR]: Disable motion detection for %d seconds(s).\n", pir_sensor.status(PIR_DEACTIVATION_DELAY));
	}
}

struct timer*
pir_get_timer()
{
	return &(pir_timer.etimer.timer);
}

/** Instantiation of the pir sensor object in memory. */
SENSORS_SENSOR(pir_sensor, PIR_SENSOR, NULL, configure, status);

/** @} */
