/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-buttons Buttons Driver
 *
 * @{
 *
 * \file
 * Header file for the button driver
 *
 * \author
 * Darko Petrovic
 *
 */
#ifndef BUTTON_SENSOR_H_
#define BUTTON_SENSOR_H_

#include "lib/sensors.h"
#include "dev/gpio.h"

#define BUTTON_SENSOR 	"Button"

#define button_sensor button_select_sensor
extern const struct sensors_sensor button_select_sensor;
extern const struct sensors_sensor button_user_sensor;
extern const struct sensors_sensor usb_plug_detect;

/** \brief Common initialiser for all buttons */
void button_sensor_init();

#endif /* BUTTON_SENSOR_H_ */

/**
 * @}
 * @}
 */
