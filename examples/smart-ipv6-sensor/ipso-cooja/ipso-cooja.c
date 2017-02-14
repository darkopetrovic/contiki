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

#if SHELL && (CONTIKI_TARGET_Z1 || CONTIKI_TARGET_WISMOTE)
#include "dev/uart1.h"
#endif

#include "dev/button-sensor.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if WITH_OMA_LWM2M
#include "lwm2m-engine.h"
#include "ipso-objects.h"
#endif


#ifndef REGISTER_WITH_LWM2M_BOOTSTRAP_SERVER
#define REGISTER_WITH_LWM2M_BOOTSTRAP_SERVER 0
#endif

#ifndef REGISTER_WITH_LWM2M_SERVER
#define REGISTER_WITH_LWM2M_SERVER 1
#endif

#ifndef LWM2M_SERVER_ADDRESS
#define LWM2M_SERVER_ADDRESS "bbbb::72"
#endif


#if SHELL
#include "dev/serial-line.h"
#include "apps/shell/shell.h"
#include "apps/serial-shell/serial-shell.h"
#include "dev/watchdog.h"
#endif /* SHELL */

#if APPS_POWERTRACK
#include "power-track.h"
#endif

#if WITH_OMA_LWM2M
static void
setup_lwm2m_servers(void)
{
#ifdef LWM2M_SERVER_ADDRESS
  uip_ipaddr_t addr;
  if(uiplib_ipaddrconv(LWM2M_SERVER_ADDRESS, &addr)) {
    lwm2m_engine_register_with_bootstrap_server(&addr, 0);
    lwm2m_engine_register_with_server(&addr, 0);
  }
#endif /* LWM2M_SERVER_ADDRESS */

  lwm2m_engine_use_bootstrap_server(REGISTER_WITH_LWM2M_BOOTSTRAP_SERVER);
  lwm2m_engine_use_registration_server(REGISTER_WITH_LWM2M_SERVER);
}
#endif

/*---------------------------------------------------------------------------*/

PROCESS(ipso_objects_process, "IPSO object process");
AUTOSTART_PROCESSES(&ipso_objects_process);

PROCESS_THREAD(ipso_objects_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("Starting as ");

#if CONF_6LOWPAN_ND
  PRINTF("6lowpan-nd ");
#else
  PRINTF("standard ");
#endif

  PROCESS_PAUSE();

#if DEBUG
  if(NODE_TYPE_ROUTER){
    PRINTF("ROUTER");
  } else {
    PRINTF("HOST");
  }

#if RPL_LEAF_ONLY
  PRINTF(" (LEAF)");
#endif
  PRINTF(".\n");
#endif

  PRINTF("Starting IPSO objects process\n");

#if SHELL
// we use the Z1 platform in cooja
#if (CONTIKI_TARGET_Z1 || CONTIKI_TARGET_WISMOTE)
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
#endif
  serial_shell_init();
  shell_ifconfig_init();
#if !CONTIKI_TARGET_Z1 && !CONTIKI_TARGET_WISMOTE
  shell_file_init();
  shell_coffee_init();
#endif
#endif /* SHELL */

#if WITH_OMA_LWM2M
#if UIP_CONF_ROUTER
  lwm2m_engine_update_registration(LWM2M_REG_LIFETIME_ROUTER, "U");
#else
  lwm2m_engine_update_registration(LWM2M_REG_LIFETIME_HOST, "UQ");
#endif

  /* Initialize the OMA LWM2M engine */
  lwm2m_engine_init();

  /* Register default LWM2M objects */
  lwm2m_engine_register_default_objects();

  /* Register default IPSO objects */
  ipso_objects_init();

  setup_lwm2m_servers();
#endif


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
