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

#include <string.h>

#if APPS_COAPSERVER
#include "rest-engine.h"
#include "custom-coap.h"
#endif

#if SHELL && CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#endif

#include "dev/button-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if SHELL
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
  res_humidity
  ;
#endif

#if APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
#if APPS_POWERTRACK
  if( !strncmp(p->name, "energest_enable", strlen(p->name)) ){
    if(p->value){
      powertrack_start(10*CLOCK_SECOND);
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

PROCESS(erbium_server, "CoAP Example");
AUTOSTART_PROCESSES(&erbium_server);

PROCESS_THREAD(erbium_server, ev, data)
{

  PROCESS_BEGIN();

  //PROCESS_PAUSE();

#ifndef CONTIKI_TARGET_CC2538DK
  SENSORS_ACTIVATE(button_sensor);
#endif

#if APPS_APPCONFIG
  app_config_init();
#if APPS_POWERTRACK
  app_config_create_parameter(APP_CONFIG_GENERAL, "energest_enable", "1", callback);
#endif /* APPS_POWERTRACK */
#endif /* APPS_APPCONFIG */

#if SHELL
// we use the Z1 platform in cooja
#ifdef CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
  serial_line_init();
#endif
  serial_shell_init();
  shell_ifconfig_init();
#if !CONTIKI_TARGET_Z1
  shell_file_init();
  shell_coffee_init();
#endif
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
#if APPS_POWERTRACK
  rest_activate_resource(&res_energest,     "energest");
#endif /* APPS_POWERTRACK */
#endif /* REST */

  /* Define application-specific events here. */
  while(1) {
    PROCESS_WAIT_EVENT();

    if( ev == sensors_event ) {
      if(data == &button_sensor) {
        PRINTF("Button select pushed.\n");

#if RDC_SLEEPING_HOST
        crdc_period_start(10);
#endif /* RDC_SLEEPING_HOST */
      }
    }

  }  /* while (1) */

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
