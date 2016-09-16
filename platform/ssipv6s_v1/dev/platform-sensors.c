/**
 * \addtogroup sensors
 * @{
 *
 *
 * \file
 * Implementation of a generic module controlling the platform sensors
 */

#include "contiki.h"

#include "platform-sensors.h"
#include "ina3221-sensor.h"
#include "pir-sensor.h"
#include "sht21-sensor.h"
#include "tmp100-sensor.h"
#include "button-sensor.h"

#include <string.h>

void
deep_sleep_ms(uint32_t duration)
{
  uint32_t i;
  /* Prevent entering PM2 mode if USB is plugged. */
  if( USB_IS_PLUGGED() ){
    for(i=0;i<duration;i++){
      clock_delay_usec(1000);
    }
  } else {
    rtimer_arch_schedule(RTIMER_NOW()+(uint32_t)(((float)(duration)/1000.0)*RTIMER_SECOND));
    REG(SYS_CTRL_PMCTL) = SYS_CTRL_PMCTL_PM2;
    ENERGEST_OFF(ENERGEST_TYPE_CPU);
    ENERGEST_ON(ENERGEST_TYPE_LPM);
    do { asm("wfi"::); } while(0);
  }
}

/* Battery voltage in mV */
uint16_t
get_battery_voltage(void)
{
  uint16_t level;
  deep_sleep_ms(125);
  SENSORS_ACTIVATE(ina3221_sensor);
  ina3221_sensor.value(INA3221_MEASUREMENT);
  level = ina3221_sensor.value(INA3221_CH2_BUS_VOLTAGE);
  SENSORS_DEACTIVATE(ina3221_sensor);
  return level;

}

/** \brief Exports a global symbol to be used by the sensor API */
SENSORS(&button_select_sensor, &button_user_sensor, &usb_plug_detect, &pir_sensor,
		    &ina3221_sensor, &sht21_sensor, &tmp100_sensor);

/**
 * @}
 */
