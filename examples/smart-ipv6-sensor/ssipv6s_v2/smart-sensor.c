/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** \addtogroup
 * @{
 *
 * \defgroup
 *
 *
 * @{
 *
 * \file
 *          Example project to test 6lowpan-nd with rpl.
 *          (for cc2538 and z1 (cooja) platforms)
 *
 */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "pir-sensor.h"
#include "platform-sensors.h"
#include "dev/leds.h"

#include <string.h>

#if APPS_COAPSERVER
#include "rest-engine.h"
#include "custom-coap.h"
#endif

#include "dev/button-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if SHELL && !USB_SHELL_IN_NRMEM
#include "dev/serial-line.h"
#include "apps/shell/shell.h"
#include "apps/serial-shell/serial-shell.h"
#include "dev/watchdog.h"
#endif /* SHELL */

#if APPS_APPCONFIG
#include "app-config.h"
#endif

#if APPS_POWERTRACK
#include "power-track.h"
#endif

#if APPS_COAPSERVER
extern resource_t
#if APPS_APPCONFIG
  res_config,
#endif
#if APPS_POWERTRACK
  res_energest,
#endif
  res_sensors,
  res_temperature,
  res_humidity,
  res_pressure,
  res_light,
  res_motion,
  res_power,
  res_battery,
  res_solar
  ;
#endif

#if APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
#if APPS_POWERTRACK
  if( !strncmp(p->name, "energest_enable", strlen(p->name)) ){
    if(p->value){
      powertrack_start(15*CLOCK_SECOND);
    } else {
      powertrack_stop();
      powertrack_reset();
    }
    return 0;
  }
#endif
  return 1;
}
#endif /* REST_DELAY_RES_START */

/*---------------------------------------------------------------------------*/

PROCESS(controller_process, "Controller process");
AUTOSTART_PROCESSES(&controller_process);

PROCESS_THREAD(controller_process, ev, data)
{

  PROCESS_BEGIN();

  //PROCESS_PAUSE();

#ifndef CONTIKI_TARGET_CC2538DK
  SENSORS_ACTIVATE(button_sensor);
#endif

  /* Execute the USB plug event to detect if usb is plugged in after
   * a software reset. */
  process_post(&controller_process, sensors_event,  (void *)&usb_plug_detect);

#if APPS_APPCONFIG
  app_config_init();
#if APPS_POWERTRACK
  app_config_create_parameter(APP_CONFIG_GENERAL, "energest_enable", "1", callback);
#endif /* APPS_POWERTRACK */
#endif /* APPS_APPCONFIG */

#if SHELL && !USB_SHELL_IN_NRMEM
  serial_shell_init();
  shell_ping_init();
  //shell_power_init();
  shell_ps_init();
  //shell_config_init();
  shell_ifconfig_init();
  //shell_stackusage_init();
  shell_file_init();
  shell_coffee_init();
#endif /* SHELL */

#if APPS_COAPSERVER
  PRINTF("Starting CoAP Server\n");
  
  /* Initialize the REST engine. */
  rest_init_engine();
#if APPS_APPCONFIG
  rest_activate_resource(&res_config,       CONFIG_RESOURCE_NAME);
#endif /* APPS_APPCONFIG */
  rest_activate_resource(&res_sensors,      "sensors");
  rest_activate_resource(&res_temperature,  "sensors/temperature");
  rest_activate_resource(&res_humidity,     "sensors/humidity");
  rest_activate_resource(&res_pressure,     "sensors/pressure");
  rest_activate_resource(&res_light,        "sensors/light");
  rest_activate_resource(&res_motion,       "sensors/motion");
  rest_activate_resource(&res_power,        "power");
  rest_activate_resource(&res_battery,      "power/battery");
  rest_activate_resource(&res_solar,        "power/solar");
#if APPS_POWERTRACK
  rest_activate_resource(&res_energest,     "energest");
#endif /* APPS_POWERTRACK */
#endif /* REST */

  /* Define application-specific events here. */
  while(1) {
    PROCESS_WAIT_EVENT();

    if( ev == sensors_event ) {
      if(data == &button_user_sensor){
        PRINTF("Button user pushed.\n");

#if RDC_SLEEPING_HOST
        crdc_period_start(30);
#endif /* RDC_SLEEPING_HOST */
      }
    }

    if(data == &usb_plug_detect){
      if(USB_IS_PLUGGED()){
        leds_on(LEDS_YELLOW);
#if RDC_SLEEPING_HOST
        /* If RDC is already enabled when the USB cable is plugged, we clear the timer
         * which is supposed to stop the RDC, and enable RDC indefinetely (while the usb
         * is plugged in). */
        crdc_clear_stop_rdc_timer();
        crdc_enable_rdc();
#endif
      } else {
        leds_off(LEDS_YELLOW);
#if RDC_SLEEPING_HOST
        crdc_disable_rdc(0);
#endif
      }

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
      usb_shell_init();
#endif
    }


    if(ev == sensors_event && data == &pir_sensor) {
      PRINTF("******* MOTION DETECTED *******\n");

#if APPS_COAPSERVER
      /* Call the event_handler for this application-specific event. */
      res_motion.trigger();
#endif
    }

  }  /* while (1) */

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
