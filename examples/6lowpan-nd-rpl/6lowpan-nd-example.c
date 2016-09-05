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
 *
 */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>

#include "dev/button-sensor.h"

#if CONTIKI_TARGET_CC2538DK
#include "usb/usb-serial.h"
#endif

#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#endif

#if SHELL
#include "dev/serial-line.h"
#include "apps/shell/shell.h"
#include "apps/serial-shell/serial-shell.h"
#include "dev/watchdog.h"
#endif /* SHELL */

#include "custom-rdc.h"
#include "apps/smart-blink/smart-blink.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if CONTIKI_TARGET_Z1
/* Used to verify network status and turn on the leds in cooja.
 * Note that green is blue, red is green in the network view.
 */
#include "dev/leds.h"
#include "net/ipv6/uip-ds6.h"
#if UIP_CONF_IPV6_RPL 
#include "net/rpl/rpl-private.h"
#endif
#define LEDS_COOJA_BLUE     LEDS_GREEN
#define LEDS_COOJA_GREEN    LEDS_RED
#define LEDS_COOJA_RED      LEDS_YELLOW
#endif /* CONTIKI_TARGET_Z1 */

/*---------------------------------------------------------------------------*/
PROCESS(nd_optimization_example, "6lowpan-nd example");
AUTOSTART_PROCESSES(&nd_optimization_example);

#if UDPCLIENT
PROCESS_NAME(udp_client_process);
#endif

#if SHELL
PROCESS(shell_fast_reboot_process, "reboot");
SHELL_COMMAND(fast_reboot_command,
        "reboot",
        "reboot: reboot the system",
        &shell_fast_reboot_process);
#endif /* SHELL */

static struct etimer status_timer;

#if CONTIKI_TARGET_Z1 && !UIP_CONF_ROUTER
static struct etimer delay_start;
#endif

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(nd_optimization_example, ev, data)
{

#if CONTIKI_TARGET_Z1
  uip_ds6_nbr_t *nbr;
  uip_ipaddr_t *defrt_addr;
#endif

  PROCESS_BEGIN();

  PRINTF("Starting as ");

#if CONF_6LOWPAN_ND
  PRINTF("6lowpan-nd ");
#else
  PRINTF("standard ");
#endif

#if DEBUG
  if(NODE_TYPE_ROUTER){
    PRINTF("ROUTER.\n");
  } else {
    PRINTF("HOST.\n");
  }
#endif

#if SHELL
// we use the Z1 platform in cooja
#ifdef CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
  serial_line_init();
#endif
  serial_shell_init();
  shell_register_command(&fast_reboot_command);

  shell_ping_init();
  //shell_power_init();
  shell_ps_init();
  //shell_config_init();
  shell_ifconfig_init();
  //shell_stackusage_init();
  //shell_file_init();
  //shell_coffee_init();
#endif /* SHELL */

#ifndef CONTIKI_TARGET_CC2538DK
  SENSORS_ACTIVATE(button_sensor);
#endif

/* Delay the start of the host. Wait network to setup properly. */
#if CONTIKI_TARGET_Z1 && !UIP_CONF_ROUTER

  etimer_stop(&uip_ds6_timer_rs);

  etimer_set(&delay_start, CLOCK_SECOND*30);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&delay_start));

  PROCESS_CONTEXT_BEGIN(&tcpip_process);
  etimer_set(&uip_ds6_timer_rs,
                 random_rand() % (UIP_ND6_RTR_SOLICITATION_INTERVAL *
                                  CLOCK_SECOND));
  PROCESS_CONTEXT_END(&tcpip_process);
#endif

#if CONTIKI_TARGET_Z1
  etimer_set(&status_timer, CLOCK_SECOND*10);
#endif

#if UDPCLIENT
  process_start(&udp_client_process, NULL);
#endif

  
  while(1) {
    PROCESS_YIELD();

#if CONTIKI_TARGET_Z1
    /* ------------------------------------------------------------------------------------ *
     * Show network status in cooja with the leds.                                          *
     * Blue led starts blinking when the node found a router. Then turn-on permanently      *
     * when registered successfully to this router when receiving NA with status 0.         *
     * Green led starts blinking when the node send DIS message and turn-on permanently     *
     * when successfully joined a DAG instance.                                             *
     * ------------------------------------------------------------------------------------ */
    if( ev == PROCESS_EVENT_TIMER )
    {
      if( data == &status_timer &&
        etimer_expired(&status_timer) )
      {

        defrt_addr = uip_ds6_defrt_choose();

        /* Node found a router. */
        if(defrt_addr != NULL){
          blink_leds(LEDS_COOJA_BLUE, CLOCK_SECOND, 0);
        } else {
          leds_off(LEDS_COOJA_BLUE);
        }

        /* Node received an NA message and successfully registered 
         * to the router (6lowpan-nd) or resolved address with this latter (ndp). */
        nbr = uip_ds6_nbr_lookup(defrt_addr);
#if CONF_6LOWPAN_ND
        if(nbr != NULL && nbr->state == NBR_REGISTERED)
#else /* CONF_6LOWPAN_ND */
        if(nbr != NULL && (nbr->state == NBR_REACHABLE || nbr->state == NBR_STALE))
#endif /* CONF_6LOWPAN_ND */
        {
          blink_leds_stop();
          leds_on(LEDS_COOJA_BLUE);
        } else {
          leds_off(LEDS_COOJA_BLUE);
        }
#if UIP_CONF_IPV6_RPL
        /* Find that a DIS message is going to be send. */
        if(rpl_get_any_dag() == NULL && rpl_get_next_dis() >= RPL_DIS_INTERVAL-1){
          blink_leds(LEDS_COOJA_GREEN, CLOCK_SECOND, 0);
        }

        /* Node joined a DODAG instance. */
        if(rpl_get_any_dag() != NULL){
          leds_on(LEDS_COOJA_GREEN);
        } else {
          leds_off(LEDS_COOJA_GREEN);
        }
#endif

        etimer_restart(&status_timer);
      }
    }
    /* ----------------------------------------------------------------------------------- */
#endif /* CONTIKI_TARGET_Z1 */

    if( ev == sensors_event ) {
      if(data == &button_sensor) {
        PRINTF("Button select pushed.\n");

#if ENABLE_CUSTOM_RDC
        crdc_period_start(10);
#endif /* ENABLE_CUSTOM_RDC */

#if UIP_CONF_DYN_HOST_ROUTER
        if(node_type==ROUTER){
          set_node_type(HOST);
#if ENABLE_CUSTOM_RDC
          crdc_disable_rdc(0);
#endif /* ENABLE_CUSTOM_RDC */
          PRINTF("Device set as HOST.\n");
        } else if(node_type==HOST){
#if ENABLE_CUSTOM_RDC
          crdc_enable_rdc();
#endif /* ENABLE_CUSTOM_RDC */
          set_node_type(ROUTER);
          PRINTF("Device set as ROUTER.\n");
        }
#endif /* UIP_CONF_DYN_HOST_ROUTER */

      }
    }
  } /* while(1) */

  PROCESS_END();
}

#if SHELL
PROCESS_THREAD(shell_fast_reboot_process, ev, data)
{
  PROCESS_BEGIN();

  shell_output_str(&fast_reboot_command,
       "Rebooting the node...", "");

  watchdog_reboot();

  PROCESS_END();
}
#endif /* SHELL */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
