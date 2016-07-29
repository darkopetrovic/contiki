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
#endif /* CONTIKI_TARGET_CC2538DK */

#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#endif

#if SHELL
#include "dev/serial-line.h"
#include "apps/shell/shell.h"
#include "apps/serial-shell/serial-shell.h"
#include "dev/watchdog.h"
#endif /* SHELL */

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
PROCESS(nd_optimization_example, "6lowpan-nd example");
AUTOSTART_PROCESSES(&nd_optimization_example);

#if SHELL
PROCESS(shell_fast_reboot_process, "reboot");
SHELL_COMMAND(fast_reboot_command,
        "reboot",
        "reboot: reboot the system",
        &shell_fast_reboot_process);
#endif /* SHELL */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(nd_optimization_example, ev, data)
{

  PROCESS_BEGIN();
  PRINTF("Starting 6lowpan-nd-rpl example process\n");

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

  while(1) {
    PROCESS_YIELD();

    if( ev == sensors_event ) {
      if(data == &button_sensor) {
#if UIP_CONF_DYN_HOST_ROUTER
        if(node_type==ROUTER){
          set_node_type(HOST);
          PRINTF("Device set as HOST.\n");
        } else if(node_type==HOST){
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
