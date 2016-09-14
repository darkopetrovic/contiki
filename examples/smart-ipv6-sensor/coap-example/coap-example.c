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
#include "custom-coap.h"

#include <string.h>

#ifdef REST
#include "rest-engine.h"
#endif
#include "dev/button-sensor.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if SHELL
#include "dev/serial-line.h"
#include "apps/shell/shell.h"
#include "apps/serial-shell/serial-shell.h"
#include "dev/watchdog.h"
#endif /* SHELL */

#if APP_CONFIG
#include "app-config.h"
#endif

#ifdef REST
extern resource_t
#if APP_CONFIG
  res_config,
#endif
  res_sensors,
  res_temperature,
  res_humidity
  ;
#endif

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

#if APP_CONFIG
  app_config_init();
#endif

#if SHELL
// we use the Z1 platform in cooja
#ifdef CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
  serial_line_init();
#endif
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

#ifdef REST
  PRINTF("Starting CoAP Server\n");
  
  /* Initialize the REST engine. */
  rest_init_engine();
#if APP_CONFIG
  rest_activate_resource(&res_config,       SETTINGS_RESOURCE_NAME);
#endif
  rest_activate_resource(&res_sensors,      "sensors");
  rest_activate_resource(&res_temperature,  "sensors/temperature");
  rest_activate_resource(&res_humidity,     "sensors/humidity");
#endif

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
