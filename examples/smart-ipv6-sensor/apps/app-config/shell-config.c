/*
 * Copyright (c) 2004, Adam Dunkels.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "contiki.h"
#include "shell.h"
#include "lib/list.h"
#include "net/ip/uip-debug.h"
#include "app-config.h"
#include "net/netstack.h"


/*---------------------------------------------------------------------------*/
PROCESS(shell_config_process, "config");
SHELL_COMMAND(config_command,
	      "config",
	      "config (print|set) <context> <name> <value>: show and edit configuration",
	      &shell_config_process);
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(shell_config_process, ev, data)
{
  char *pch1, *pch2, *pch3;
  char *saveptr;
  char outputbuf[100];
  uint8_t nbsettings;
  struct parameter* p;
  uint32_t value;
  radio_value_t radioval;

  PROCESS_BEGIN();

  pch1 = strtok_r(data, " ", &saveptr);

  if( !strcmp(pch1, "print") )
  {
    // read <context> positional argument
    pch1 = strtok_r(NULL, " ", &saveptr);

    if( pch1 != NULL ){

      snprintf(outputbuf, 100, "Parameters in context '%s':", pch1);
      shell_output_str(&config_command, outputbuf, "");

      nbsettings = 0;
      for(p = app_config_parameters_list_head(); p != NULL; p = list_item_next(p))
      {
        if( !strcmp (pch1, p->context) )
        {
          if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "PANID")){
            NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &radioval);
            snprintf(outputbuf, 100, "    %-20s %-20s 0x%04X", p->context, p->name, radioval);
          } else if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "txpower")){
            NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &radioval);
            snprintf(outputbuf, 100, "    %-20s %-20s %ddBm", p->context, p->name, radioval);
          } else if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "ccathreshold")){
            NETSTACK_RADIO.get_value(RADIO_PARAM_CCA_THRESHOLD, &radioval);
            snprintf(outputbuf, 100, "    %-20s %-20s %ddBm", p->context, p->name, (int8_t)radioval);
          } else if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "channel")){
            NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &radioval);
            snprintf(outputbuf, 100, "    %-20s %-20s %d", p->context, p->name, (uint8_t)radioval);
          }

          else {
            if(p->is_string){
              snprintf(outputbuf, 100, "    %-20s %-20s %s", p->context, p->name,
                  (const char*)app_config_get_parameter_value(p->context, p->name));
            } else {
              snprintf(outputbuf, 100, "    %-20s %-20s %lu", p->context, p->name, p->value);
            }
          }

          shell_output_str(&config_command, outputbuf, "");
          nbsettings++;
        }
      }

      if( !nbsettings ){
        snprintf(outputbuf, 100, "(error) Parameter context '%s' doesn't exist.", pch1);
        shell_output_str(&config_command, outputbuf, "");
      }
    } else {
      shell_output_str(&config_command, "Parameters list:", "");
      shell_output_str(&config_command, "    Context              Name                  Value", "");
      shell_output_str(&config_command, "    --------------------------------------------------", "");

      for(p = app_config_parameters_list_head(); p != NULL; p = list_item_next(p))
      {
        if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "PANID")){
          NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, &radioval);
          snprintf(outputbuf, 100, "    %-20s %-20s 0x%04X", p->context, p->name, radioval);
        } else if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "txpower")){
          NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &radioval);
          snprintf(outputbuf, 100, "    %-20s %-20s %ddBm", p->context, p->name, radioval);
        } else if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "ccathreshold")){
          NETSTACK_RADIO.get_value(RADIO_PARAM_CCA_THRESHOLD, &radioval);
          snprintf(outputbuf, 100, "    %-20s %-20s %ddBm", p->context, p->name, (int8_t)radioval);
        } else if(!strncmp(p->context, "radio", 5) && !strcmp(p->name, "channel")){
          NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &radioval);
          snprintf(outputbuf, 100, "    %-20s %-20s %d", p->context, p->name, (uint8_t)radioval);
        }

        else {
          if(p->is_string){
            snprintf(outputbuf, 100, "    %-20s %-20s %s", p->context, p->name,
                (const char*)app_config_get_parameter_value(p->context, p->name));
          } else {
            snprintf(outputbuf, 100, "    %-20s %-20s %lu", p->context, p->name, p->value);
          }
        }

        shell_output_str(&config_command, outputbuf, "");
        nbsettings++;
      }
    }
  }

  else if( !strcmp(pch1, "set") )
  {
    // read <context> positional argument
    pch1 = strtok_r(NULL, " ", &saveptr);
    // read <name> positional argument
    pch2 = strtok_r(NULL, " ", &saveptr);
    p = app_config_parameter_lookup( (const char*)pch1, (const char*)pch2 );

    if( p != NULL ){
      // read <value> positional argument
      pch3 = strtok_r(NULL, " ", &saveptr);

      snprintf(outputbuf, 100, "Update parameter '%s - %s' = %s.",
                (const char*)pch1, (const char*)pch2, (const char*) pch3);

      /* Convert hexadecimal string to decimal */
      if(!strncmp(pch3, "0x", 2)){
        value = (uint32_t)strtol(pch3, NULL, 0);
        // since the priority in on the string value when using the function app_config_edit_parameter()
        // we have to set the string value to null
        pch3 = NULL;
      } else {
        value = 0;
      }

      if(!app_config_edit_parameter((const char*)pch1, (const char*)pch2, (const char*) pch3, value))
      {
        shell_output_str(&config_command, outputbuf, "");
      } else {
        snprintf(outputbuf, 100, "(error) Update parameter '%s' in context '%s' failed.", pch2, pch1);
        shell_output_str(&config_command, outputbuf, "");
      }

    } else {
      snprintf(outputbuf, 100, "(error) Parameter '%s' in context '%s' not found.", pch2, pch1);
      shell_output_str(&config_command, outputbuf, "");
    }
  }

  else {
    shell_output_str(&config_command, "Parameter doesn't exist (use print or set).", "");
  }

  PROCESS_END();
  }
/*---------------------------------------------------------------------------*/
void
shell_config_init(void)
{
  shell_register_command(&config_command);
}
/*---------------------------------------------------------------------------*/
