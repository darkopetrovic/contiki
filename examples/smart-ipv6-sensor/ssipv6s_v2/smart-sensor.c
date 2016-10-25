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
#include "mic-sensor.h"
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

#if APPS_SMARTLED
#include "smart-led.h"
#endif

static struct ctimer microclap_timer;
static rtimer_clock_t button_time_press;
static rtimer_clock_t button_time_release;
static uint8_t starting;

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
  res_power,
  res_battery,
  res_solar,
  res_events,
  res_micro,
  res_motion
  ;
#endif

extern uint8_t res_micro_clap_counter;

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

static void
microclap_timeout(void* ptr)
{
#if APPS_COAPSERVER
  res_micro.trigger();
  res_events.trigger();
#endif

  res_micro_clap_counter = 0;
}

#if DEBUG
static char *
float2str(float num, uint8_t preci)
{
  int integer=(int)num, decimal=0;
  static char buf[20];
  preci = preci > 10 ? 10 : preci;
  num -= integer;
  while((num != 0) && (preci-- > 0)) {
    decimal *= 10;
    num *= 10;
    decimal += (int)num;
    num -= (int)num;
  }
  switch(preci){
    case 1:
      sprintf(buf, "%d.%01d", integer, decimal);
      break;
    case 2:
      sprintf(buf, "%d.%02d", integer, decimal);
      break;
    case 3:
      sprintf(buf, "%d.%03d", integer, decimal);
      break;
    default:
      sprintf(buf, "%d.%01d", integer, decimal);
  }
  return buf;
}
#endif

static void
button_press_action( rtimer_clock_t delta )
{
  float button_press_duration;

  button_press_duration = (float)(delta)/RTIMER_SECOND;

  PRINTF("Button pressed during %s second(s).\n",
        float2str(button_press_duration, 2));

  if( button_press_duration >= 8){
    // reset the device
    // ...
  } else if ( button_press_duration < 8 && button_press_duration >= 3) {
    // become a router or an host
#if UIP_CONF_DYN_HOST_ROUTER
    if(NODE_TYPE_HOST){
      app_config_edit_parameter(APP_CONFIG_GENERAL, "router", NULL, 1);
    } else {
      app_config_edit_parameter(APP_CONFIG_GENERAL, "router", NULL, 0);
    }
#endif
  }
}

#if RDC_SLEEPING_HOST
static uint8_t
change_rdc_period(struct parameter *p)
{
  // don't start rdc at device startup
  if( !starting ){
    crdc_disable_rdc(0);
    crdc_period_start( p->value );
  }
  return 0;
}
#endif

#if UIP_CONF_DYN_HOST_ROUTER
static uint8_t
node_change_type(struct parameter *p)
{
  /* Configure the device as a router or host.
   * This function is exectued whenever we update the parameter
   * and at startup once the setting is read from flash. */
  if(node_type != p->value){

    node_type = p->value;
    if(NODE_TYPE_ROUTER && USB_IS_PLUGGED()){
      set_node_type(ROUTER);
      PRINTF("Node set as ROUTER.\n");
#if APPS_SMARTLED
        if(!starting){
          blink_leds(LEDS_YELLOW, CLOCK_SECOND/4, 3);
        }
#endif
      return 0;
    } else {
      /* The device fails to become a router. */
      if(NODE_TYPE_ROUTER){
        /* If the device is set as router and the usb cable isn't plugged in
         * at startup, the device is automatically set as an host but the
         * setting is unchanged.
         */
        node_type = HOST;
#if APPS_SMARTLED
        if(!starting){
          blink_leds(LEDS_RED, CLOCK_SECOND/4, 3);
        }
#endif
        return 1;
      }

      /* This function doesn't need to be executed at startup since
       * the device is by default an host. */
      if(!starting){
        set_node_type(HOST);
#if RDC_SLEEPING_HOST
        if(!USB_IS_PLUGGED()){
          crdc_disable_rdc(0);
        }
#endif
        PRINTF("Node set as HOST.\n");
#if APPS_SMARTLED
        blink_leds(LEDS_YELLOW, CLOCK_SECOND/2, 3);
#endif
      }
      return 0;
    }
  }
  /* Don't return as an error. We want the parameter to
   * be stored in flash. */
  return 0;
}
#endif /* UIP_CONF_DYN_HOST_ROUTER */

/*---------------------------------------------------------------------------*/

PROCESS(controller_process, "Controller process");
AUTOSTART_PROCESSES(&controller_process);

PROCESS_THREAD(controller_process, ev, data)
{

  PROCESS_BEGIN();

  //PROCESS_PAUSE();

  starting = 1;

#ifndef CONTIKI_TARGET_CC2538DK
  SENSORS_ACTIVATE(button_sensor);
#endif

  /* Execute the USB plug event to detect if usb is plugged in after
   * a software reset. */
  process_post(&controller_process, sensors_event,  (void *)&usb_plug_detect);

#if APPS_APPCONFIG
  app_config_init();

#if UIP_CONF_DYN_HOST_ROUTER
#if UIP_CONF_ROUTER
  app_config_create_parameter(APP_CONFIG_GENERAL, "router", "1", node_change_type);
#else
  app_config_create_parameter(APP_CONFIG_GENERAL, "router", "0", node_change_type);
#endif
#endif

#if RDC_SLEEPING_HOST
  app_config_create_parameter(APP_CONFIG_GENERAL, "rdc_enable_period", "30", change_rdc_period);
#endif

  app_config_create_parameter(APP_CONFIG_GENERAL, "alive_message_period", "30", NULL);
  app_config_create_parameter(APP_CONFIG_GENERAL, "bripaddr", "aaaa::212:4b00:40e:fadb", NULL);

#if CONF_6LOWPAN_ND
  /* The time the host will be registered to the router. */
  app_config_create_parameter(APP_CONFIG_GENERAL, "aro-registration", "10", NULL);
#endif

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
  rest_activate_resource(&res_power,        "power");
  rest_activate_resource(&res_battery,      "power/battery");
  rest_activate_resource(&res_solar,        "power/solar");
  rest_activate_resource(&res_events,       "events");
  rest_activate_resource(&res_motion,       "events/motion");
  rest_activate_resource(&res_micro,        "events/micro");
#if APPS_POWERTRACK
  rest_activate_resource(&res_energest,     "energest");
#endif /* APPS_POWERTRACK */
#endif /* REST */

  starting = 0;

  /* Define application-specific events here. */
  while(1) {
    PROCESS_WAIT_EVENT();

    if(ev == sensors_event) {
      /* =========== USER BUTTON ============== */
      if(data == &button_user_sensor){
        PRINTF("Button user pushed.\n");
#if RDC_SLEEPING_HOST
#if USB_SERIAL_CONF_ENABLE
        if( !USB_IS_PLUGGED() ){
          // the RDC is activated only for few seconds/minutes
          crdc_period_start( *(uint32_t*)app_config_get_parameter_value(APP_CONFIG_GENERAL, "rdc_enable_period") );
        }
#else /* USB_SERIAL_CONF_ENABLE */
        if( !button_time_press ){
          crdc_period_start(  *(uint32_t*)app_config_get_parameter_value(APP_CONFIG_GENERAL, "rdc_enable_period") );
        }
#endif /* USB_SERIAL_CONF_ENABLE */
#endif /* RDC_SLEEPING_HOST */

        if( !button_time_press ){
          button_time_press = RTIMER_NOW();
        } else {
          button_time_release = RTIMER_NOW();
        }

        if( button_time_release ){
          button_press_action( (button_time_release - button_time_press) );
          button_time_press = 0;
          button_time_release = 0;
        }
      }

      /* =========== USB PLUG ============== */
      if(data == &usb_plug_detect){
        if(USB_IS_PLUGGED()){
          leds_on(LEDS_YELLOW);
#if RDC_SLEEPING_HOST
          /* If RDC is already enabled when the USB cable is plugged, we clear the timer
           * which is supposed to stop the RDC, and enable RDC indefinetely (while the usb
           * is plugged in). */
          crdc_clear_stop_rdc_timer();
          crdc_disable_rdc(1);
#endif
        } else {
          leds_off(LEDS_YELLOW);

#if RDC_SLEEPING_HOST
          if(NODE_TYPE_HOST){
            crdc_disable_rdc(0);
          } else {
            crdc_enable_rdc();
          }
#endif

#if UIP_CONF_DYN_HOST_ROUTER
#else /* UIP_CONF_DYN_HOST_ROUTER */

#endif /* UIP_CONF_DYN_HOST_ROUTER */
        }

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
        usb_shell_init();
#endif
      }

      /* =========== MOTION DETECT ============== */
      if(data == &pir_sensor) {
        PRINTF("******* MOTION DETECTED *******\n");
#if APPS_COAPSERVER
        /* Call the event_handler for this application-specific event. */
        res_motion.trigger();
        res_events.trigger();
#endif
      }

      /* =========== MICROPHONE DETECT ============== */
      if(data == &mic_sensor) {
        PRINTF("******* SOUND DETECTED *******\n");
#if ADC_ACQUISITION_ON
        SENSORS_MEASURE(mic_sensor);
#endif
        if(res_micro_clap_counter < 4){
          res_micro_clap_counter++;
          ctimer_set(&microclap_timer, CLOCK_SECOND, microclap_timeout, NULL);
        }
      }
    } // if( ev == sensors_event )
  }  /* while (1) */

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
