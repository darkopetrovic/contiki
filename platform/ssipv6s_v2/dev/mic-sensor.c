/**
 * \addtogroup sensor-pir
 * @{
 *
 * \file
 * Driver for the PIR sensor
 *
 * \author
 * Darko Petrovic
 */

#include "mic-sensor.h"
#include "soc-adc.h"
#include "adc.h"

/** \cond */
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/** \endcond */

static struct timer debouncetimer;
static uint8_t internal_status;

#if ADC_ACQUISITION_ON
static uint16_t adcvalues[ADC_SAMPLES] __attribute__ ((section (".nrbss")));
#endif

/*!**********************************************************************************
 * \brief 		Callback registered with the GPIO module. Gets fired with a button
 * 				port/pin generates an interrupt
 *
 * \param port 	The port number that generated the interrupt
 * \param pin 	The pin number that generated the interrupt. This is the pin
 * 				absolute number (i.e. 0, 1, ..., 7), not a mask
 ***********************************************************************************/
static void
mic_callback(uint8_t port, uint8_t pin)
{

  // The delay is started the first time with the function below.
  if(!timer_expired(&debouncetimer)) {
    return;
  }

  timer_set(&debouncetimer, CLOCK_SECOND/4);

  if(port == MIC_COMP_INPUT_PORT) {
    switch(pin) {
      case MIC_COMP_INPUT_PIN:
        PRINTF("Sensor [MIC]: Sound detected!\n");
        sensors_changed(&mic_sensor);
        break;
      default:
        return;
    }
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
#if ADC_ACQUISITION_ON
  uint16_t i;
#endif

  switch(type) {
    case SENSORS_HW_INIT:

#if ADC_ACQUISITION_ON
      // configure ADC input pin
      GPIO_SOFTWARE_CONTROL(MIC_ADC_INPUT_PORT_BASE, MIC_ADC_INPUT_PIN_MASK);
      GPIO_SET_INPUT(MIC_ADC_INPUT_PORT_BASE, MIC_ADC_INPUT_PIN_MASK);
      ioc_set_over(MIC_ADC_INPUT_PORT, MIC_ADC_INPUT_PIN, IOC_OVERRIDE_ANA);
#endif /* ADC_ACQUISITION_ON */
      /* ----------------------------------------------------------------------------- */

      // configure COMPARATOR pins
      GPIO_SET_INPUT(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);
      ioc_set_over(MIC_COMP_INPUT_PORT, MIC_COMP_INPUT_PIN, IOC_OVERRIDE_ANA);

      GPIO_SET_INPUT(MIC_COMP_REF_PORT_BASE, MIC_COMP_REF_PIN_MASK);
      ioc_set_over(MIC_COMP_REF_PORT, MIC_COMP_REF_PIN, IOC_OVERRIDE_ANA);

      /* Software controlled */
      GPIO_SOFTWARE_CONTROL(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);
      /* Set pin to input */
      GPIO_SET_INPUT(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);
      /* Enable edge detection */
      GPIO_DETECT_EDGE(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);
      /* Single edge */
      GPIO_TRIGGER_BOTH_EDGES(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);
      /* Trigger interrupt on rising edge */
      //GPIO_DETECT_RISING(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);
      /* Power-up the SoC from sleep if detection occurs */
      GPIO_POWER_UP_ON_FALLING(MIC_COMP_INPUT_PORT, MIC_COMP_INPUT_PIN_MASK);
      /* Do not activate immediatly the interrupt */
      //GPIO_DISABLE_POWER_UP_INTERRUPT(MIC_COMP_INPUT_PORT, MIC_COMP_INPUT_PIN_MASK);
      GPIO_ENABLE_POWER_UP_INTERRUPT(MIC_COMP_INPUT_PORT, MIC_COMP_INPUT_PIN_MASK);
      GPIO_ENABLE_INTERRUPT(MIC_COMP_INPUT_PORT_BASE, MIC_COMP_INPUT_PIN_MASK);

      nvic_interrupt_enable(MIC_COMP_INPUT_VECTOR);

      /* Callback function when the gpio pin is triggered. */
      gpio_register_callback(mic_callback, MIC_COMP_INPUT_PORT, MIC_COMP_INPUT_PIN);

      timer_set(&debouncetimer, 0);

      break;

    case SENSORS_ACTIVE:
      if( value ){
        // turn on the comparator
        REG(SOC_ADC_CMPCTL) = SOC_ADC_CMPCTL_EN;
        MIC_CIRCUIT_ON();
      } else {
        MIC_CIRCUIT_OFF();
        REG(SOC_ADC_CMPCTL) = 0;
      }
      break;

#if ADC_ACQUISITION_ON
    case SENSORS_DO_MEASURE:
      for(i=0;i<ADC_SAMPLES;i++){
        adcvalues[i] = adc_get(SOC_ADC_ADCCON_CH_AIN0, SOC_ADC_ADCCON_REF_AVDD5, SOC_ADC_ADCCON_DIV_512);
      }
      break;
#endif /* ADC_ACQUISITION_ON */

    default:
      return 1;
  }

  return 0;
}

static int
value(int type)
{
#if ADC_ACQUISITION_ON
  return &adcvalues;
#endif /* ADC_ACQUISITION_ON */
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
    case MIC_POWER:
      if( internal_status & MIC_FLAG_POWERED ){
        return 1;
      } else {
        return 0;
      }
      break;

    default:
      return -1;
  }
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(mic_sensor, MIC_SENSOR, value, configure, status);

/** @} */
