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

#ifndef MIC_SENSOR_H_
#define MIC_SENSOR_H_

#include "lib/sensors.h"
#include "dev/ioc.h"
#include "platform-sensors.h"

#define MIC_SENSOR              "MICROPHONE"

/* -----------------------------------------------------------------*/
/*				/!\ Do not alter values below /!\					*/
/* -----------------------------------------------------------------*/

#define MIC_PWR_PORT_BASE           GPIO_PORT_TO_BASE(MIC_PWR_PORT)
#define MIC_PWR_PIN_MASK            GPIO_PIN_MASK(MIC_PWR_PIN)

#define MIC_ADC_INPUT_PORT_BASE     GPIO_PORT_TO_BASE(MIC_ADC_INPUT_PORT)
#define MIC_ADC_INPUT_PIN_MASK      GPIO_PIN_MASK(MIC_ADC_INPUT_PIN)

#define MIC_COMP_INPUT_PORT_BASE    GPIO_PORT_TO_BASE(MIC_COMP_INPUT_PORT)
#define MIC_COMP_INPUT_PIN_MASK     GPIO_PIN_MASK(MIC_COMP_INPUT_PIN)

#define MIC_COMP_REF_PORT_BASE      GPIO_PORT_TO_BASE(MIC_COMP_REF_PORT)
#define MIC_COMP_REF_PIN_MASK       GPIO_PIN_MASK(MIC_COMP_REF_PIN)

#define MIC_FLAG_POWERED            (1<<0)

/** Power-ups the Microphone sensor. */
#define MIC_CIRCUIT_ON()            ioc_set_over(MIC_COMP_REF_PORT, MIC_COMP_REF_PIN, IOC_OVERRIDE_DIS);\
                                    GPIO_SET_PIN(MIC_PWR_PORT_BASE, MIC_PWR_PIN_MASK);\
                                    internal_status |= MIC_FLAG_POWERED

/** Power-down the PIR sensor. */
#define MIC_CIRCUIT_OFF()           ioc_set_over(MIC_COMP_REF_PORT, MIC_COMP_REF_PIN, IOC_OVERRIDE_PDE);\
                                    GPIO_CLR_PIN(MIC_PWR_PORT_BASE, MIC_PWR_PIN_MASK);\
                                    internal_status &= ~(MIC_FLAG_POWERED)

#ifdef ADC_ACQUISITION_ON
#define ADC_SAMPLES                 8000
#endif

extern const struct sensors_sensor mic_sensor;

/**
 *  User defined states for the \ref configure() and \ref status() function.
 */
enum {
  MIC_POWER,        /*!< Power state of the sensor */
};

/*---------------------------------------------------------------------------*/
#endif /* MIC_SENSOR_H_ */


/**
 * @}
 * @}
 */
