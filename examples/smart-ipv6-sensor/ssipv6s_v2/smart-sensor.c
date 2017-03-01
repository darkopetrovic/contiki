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
#include "cpu.h"

#include "rpl-private.h"

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

#include "lwm2m-engine.h"

#if WITH_IPSO
#include "ipso-objects.h"
#endif

#ifndef REGISTER_WITH_LWM2M_BOOTSTRAP_SERVER
#define REGISTER_WITH_LWM2M_BOOTSTRAP_SERVER 0
#endif

#ifndef REGISTER_WITH_LWM2M_SERVER
#define REGISTER_WITH_LWM2M_SERVER 1
#endif

#ifndef LWM2M_SERVER_ADDRESS
#define LWM2M_SERVER_ADDRESS "2001::33"
#endif

#ifndef SMART_CONF_ALIVE_MSG
#define SMART_ALIVE_MSG           0
#else
#define SMART_ALIVE_MSG           SMART_CONF_ALIVE_MSG
#endif

#ifndef SMART_CONF_ALIVE_MSG_RDC_DURATION
#define SMART_ALIVE_MSG_RDC_DURATION    3
#else
#define SMART_ALIVE_MSG_RDC_DURATION    SMART_CONF_ALIVE_MSG_RDC_DURATION
#endif

#ifndef SMART_CONF_ALIVE_MSG_MULTICAST
#define SMART_ALIVE_MSG_MULTICAST     1
#else
#define SMART_ALIVE_MSG_MULTICAST     SMART_CONF_ALIVE_MSG_MULTICAST
#endif

/* Disable sending ALIVE message of course when the
 * custom RDC is disabled. */
#if !APPS_CUSTOMRDC || UIP_CONF_ROUTER
#undef SMART_ALIVE_MSG
#define SMART_ALIVE_MSG             0
#endif

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
  res_motion,
  res_air,
  res_leds
  ;
#endif

extern uint8_t res_micro_clap_counter;

#if SMART_ALIVE_MSG
static struct ctimer alive_message_timer;
#endif /* SMART_ALIVE_MSG */

#if APPS_OMALWM2M
static void
setup_lwm2m_servers(void)
{
  const char *serverip;

#if APPS_APPCONFIG
  serverip = (const char *)app_config_get_parameter_value(APP_CONFIG_GENERAL, "lwm2m-server");
#else
  serverip = LWM2M_SERVER_ADDRESS;
#endif /* APPS_APPCONFIG */

  uip_ipaddr_t addr;
  if(uiplib_ipaddrconv(serverip, &addr)) {
    lwm2m_engine_register_with_bootstrap_server(&addr, 0);
    lwm2m_engine_register_with_server(&addr, 0);
  }

  lwm2m_engine_use_bootstrap_server(REGISTER_WITH_LWM2M_BOOTSTRAP_SERVER);
  lwm2m_engine_use_registration_server(REGISTER_WITH_LWM2M_SERVER);
}
#endif /* APPS_OMALWM2M */

static void
button_press_action( rtimer_clock_t delta )
{
  float button_press_duration;

  button_press_duration = (float)(delta)/RTIMER_SECOND;

  /*PRINTF("Button pressed during %s second(s).\n",
        float2str(button_press_duration, 2));*/

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

static uint8_t
radio_params(struct parameter *p)
{
  if( !strcmp(p->name, "PANID") ){
    NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, p->value);
  } else if( !strcmp(p->name, "channel") ) {
    NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, p->value);
  } else if( !strcmp(p->name, "txpower") ) {
    NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, p->value);
  } else if( !strcmp(p->name, "ccathreshold") ) {
    NETSTACK_RADIO.set_value(RADIO_PARAM_CCA_THRESHOLD, p->value);
  }
  return 0;
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

#if SMART_ALIVE_MSG

static void
send_alive_message(void *ptr)
{
  PRINTF("SMART: Send ALV message.\n");

#ifdef LWM2M_SERVER_ADDRESS
  uip_ipaddr_t addr;
  static coap_packet_t message[1];

  coap_init_message(message, COAP_TYPE_NON, CONTENT_2_05, coap_get_mid());
  coap_set_payload(message, "ALV", 3);

  /* If RDC isn't ON here, this means that another wake-up preceded very shortly
   * the ALV wake-up. To be sure to send the packet, we turn-on the RDC here. */
  if( !crdc_get_rdc_status() ){
    PRINTF("SMART: Send ALV message (RDC off -> turn on).\n");
    crdc_period_start( SMART_ALIVE_MSG_RDC_DURATION );
  }

  uiplib_ipaddrconv(LWM2M_SERVER_ADDRESS, &addr);

  coap_send_message(&addr, UIP_HTONS(COAP_DEFAULT_PORT), uip_appdata,
      coap_serialize_message(message, uip_appdata));


#else
#if SMART_ALIVE_MSG_MULTICAST
  static coap_packet_t message[1];
  uip_ipaddr_t addr;
  coap_init_message(message, COAP_TYPE_NON, CONTENT_2_05, coap_get_mid());
  coap_set_payload(message, "ALV", 3);

  /* If RDC isn't ON here, this means that another wake-up preceded very shortly
   * the ALV wake-up. To be sure to send the packet, we turn-on the RDC here. */
  if( !crdc_get_rdc_status() ){
    PRINTF("SMART: Send ALV message (RDC off -> turn on).\n");
    crdc_period_start( SMART_ALIVE_MSG_RDC_DURATION );
  }

  // Site-Local Scope Multicast Addresses
  uip_ip6addr(&addr, 0xff05, 0, 0, 0, 0, 0, 0, 0x1);

  coap_send_message(&addr, UIP_HTONS(COAP_DEFAULT_PORT), uip_appdata,
      coap_serialize_message(message, uip_appdata));

#else
  uip_ds6_border_router_t *locbr;
  /* Send to border routers */
  for(locbr = uip_ds6_br_list;
      locbr < uip_ds6_br_list + UIP_DS6_BR_NB;
      locbr++)
  {
    /* If RDC isn't ON here, this means that another wake-up preceded very shortly
     * the ALV wake-up. To be sure to send the packet, we turn-on the RDC here. */
    if( !crdc_get_rdc_status() ){
      PRINTF("SMART: Send ALV message (RDC off -> turn on).\n");
      crdc_period_start( SMART_ALIVE_MSG_RDC_DURATION );
    }
    uip_icmp6_send(&locbr->ipaddr, 200, 0, 0);
  }
#endif /* SMART_ALIVE_MSG_MULTICAST */
#endif /* LWM2M_SERVER_ADDRESS */

  ctimer_restart(&alive_message_timer);
}

static void
start_alive_msg_periodic(void)
{
  /* Start the alive message (ALV) periodic timer. */
  ctimer_set(&alive_message_timer,
          CLOCK_SECOND *  *(uint32_t*)app_config_get_parameter_value(APP_CONFIG_GENERAL, "alive_message_period") ,
          send_alive_message, NULL);

  PRINTF("SMART: Starting ALIVE messages with a period of %us.\n",
      CLOCK_SECOND *  *(uint32_t*)app_config_get_parameter_value(APP_CONFIG_GENERAL, "alive_message_period") );
}

static uint8_t
change_alv_period(struct parameter *p)
{
  alive_message_timer.etimer.timer.interval = CLOCK_SECOND * p->value;
  return 0;
}

#endif /* SMART_ALIVE_MSG */

#if UIP_CONF_DYN_HOST_ROUTER
static uint8_t
node_change_type(struct parameter *p)
{
  PRINTF("Changing node type.\n");
  /* Configure the device as a router or host.
   * This function is exectued whenever we update the parameter
   * and at startup once the setting is read from flash. */
  if(p->value==ROUTER && USB_IS_PLUGGED()){
    set_node_type(ROUTER);
    PRINTF("Node set as ROUTER.\n");

    if(!starting){
#if APPS_SMARTLED
      blink_leds(LEDS_YELLOW, CLOCK_SECOND/4, 3);
#endif
      rpl_periodic_timer_update(1);
    }

#if SMART_ALIVE_MSG
    ctimer_stop(&alive_message_timer);
#endif
#if APPS_OMALWM2M
    lwm2m_engine_update_registration(LWM2M_REG_LIFETIME_ROUTER, "U");
#endif

    return 0;
  } else {
    /* The device fails to become a router because USB cable isn't plugged in. */
    if(p->value==ROUTER){
      /* If the device is set as router and the usb cable isn't plugged in
       * at startup, the device is automatically set as an host but the
       * setting in memory is unchanged.
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
      rpl_periodic_timer_update(RPL_PERIODIC_INTERVAL);
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

#if APPS_OMALWM2M
    lwm2m_engine_update_registration(LWM2M_REG_LIFETIME_HOST, "UQ");
#endif
#if SMART_ALIVE_MSG
    start_alive_msg_periodic();
#endif
    return 0;
  }
  /* Don't return as an error. We want the parameter to
   * be stored in flash. */
  return 0;
}
#endif /* UIP_CONF_DYN_HOST_ROUTER */

void
ds_notification_callback(int event,
         uip_ipaddr_t *route,
         uip_ipaddr_t *nexthop,
         int num_routes)
{
  if ( event == UIP_DS6_NOTIFICATION_DEFRT_ADD ) {
    //blink_leds(LEDS_YELLOW, CLOCK_SECOND/4, 4);
  } else if ( event == UIP_DS6_NOTIFICATION_DEFRT_RM ) {
    //blink_leds(LEDS_RED, CLOCK_SECOND/4, 4);
  } else if( event == UIP_DS6_NOTIFICATION_BR_ADD ) {
    blink_leds(LEDS_YELLOW, CLOCK_SECOND/4, 4);
  } else if ( event == UIP_DS6_NOTIFICATION_BR_RM ) {
    blink_leds(LEDS_RED, CLOCK_SECOND/4, 4);
  }
}

/*---------------------------------------------------------------------------*/

PROCESS(ipso_objects_process, "IPSO object process");
PROCESS(controller_process, "Controller process");
AUTOSTART_PROCESSES(&controller_process, &ipso_objects_process);

PROCESS_THREAD(controller_process, ev, data)
{
  static struct uip_ds6_notification n;

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

#if SMART_ALIVE_MSG
  app_config_create_parameter(APP_CONFIG_GENERAL, "alive_message_period", "30", change_alv_period);
#endif

#if APPS_OMALWM2M
  app_config_create_parameter(APP_CONFIG_GENERAL, "lwm2m-server", "bbbb::72", NULL);
#endif

#if CONF_6LOWPAN_ND
  /* The time the host will be registered to the router. */
  app_config_create_parameter(APP_CONFIG_GENERAL, "aro-registration", "10", NULL);
#endif

  app_config_create_parameter("radio", "PANID", "43981", radio_params); // 0xABCD
  app_config_create_parameter("radio", "channel", "25", radio_params);
  app_config_create_parameter("radio", "txpower", "-24", radio_params);
  app_config_create_parameter("radio", "ccathreshold", "175", radio_params); // -81dBm

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
  rest_activate_resource(&res_air,          "sensors/air");
  rest_activate_resource(&res_power,        "power");
  rest_activate_resource(&res_battery,      "power/battery");
  rest_activate_resource(&res_solar,        "power/solar");
  rest_activate_resource(&res_events,       "events");
  rest_activate_resource(&res_motion,       "events/motion");
  rest_activate_resource(&res_micro,        "events/micro");
  rest_activate_resource(&res_leds,         "actuator/leds");
#if APPS_POWERTRACK
  rest_activate_resource(&res_energest,     "energest");
#endif /* APPS_POWERTRACK */
#endif /* REST */

  starting = 0;

  uip_ds6_notification_add(&n, ds_notification_callback);

  /* Define application-specific events here. */
  while(1) {
    PROCESS_WAIT_EVENT();

    if(ev == sensors_event) {
      /* =========== USER BUTTON ============== */
      if(data == &button_user_sensor){
        PRINTF("Button user pushed.\n");

#if RDC_SLEEPING_HOST
        if( !USB_IS_PLUGGED() ){
#if APPS_OMALWM2M
          // by pressing the user button we update the registration
          // to receive the pending packets on the server
          lwm2m_engine_update_registration(0, NULL);
#else /* APPS_OMALWM2M */
          // the RDC is activated only for few seconds/minutes
          crdc_period_start( *(uint32_t*)app_config_get_parameter_value(APP_CONFIG_GENERAL, "rdc_enable_period") );
#endif /* APPS_OMALWM2M */
        }
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

        if( reading_voltage ){
          INTERRUPTS_DISABLE();
          nvic_interrupt_disable(USB_PLUG_DETECT_VECTOR);
          USB_REG_ENABLE();
          nvic_interrupt_unpend(USB_PLUG_DETECT_VECTOR);
          nvic_interrupt_enable(USB_PLUG_DETECT_VECTOR);
          INTERRUPTS_ENABLE();
          reading_voltage = 0;
        } else {

          if(USB_IS_PLUGGED()){
            leds_on(LEDS_YELLOW);

            if( *(uint32_t*)app_config_get_parameter_value(APP_CONFIG_GENERAL, "router") )
            {
              blink_leds(LEDS_YELLOW, CLOCK_SECOND/4, 3);
            }

  #if RDC_SLEEPING_HOST
            /* If RDC is already enabled when the USB cable is plugged, we clear the timer
             * which is supposed to stop the RDC, and enable RDC indefinetely (while the usb
             * is plugged in). */
            crdc_clear_stop_rdc_timer();
            crdc_disable_rdc(1);
            lwm2m_engine_update_registration(LWM2M_REG_LIFETIME_ROUTER, "U");
  #endif
          } else {
            leds_off(LEDS_YELLOW);

  #if RDC_SLEEPING_HOST
            if(NODE_TYPE_HOST){
              crdc_disable_rdc(0);
              lwm2m_engine_update_registration(LWM2M_REG_LIFETIME_HOST, "UQ");
            } else {
              crdc_enable_rdc();
            }
  #endif
          }

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
        usb_shell_init();
#endif

        }
      }

      /* =========== MOTION DETECT ============== */
      if(data == &pir_sensor) {
        PRINTF("******* MOTION DETECTED *******\n");
#if APPS_COAPSERVER
        /* Call the event_handler for this application-specific event. */
        res_motion.trigger();
        res_events.trigger();
#endif
#if WITH_IPSO
        ipso_presence_detection();
#endif
      }

      /* =========== MICROPHONE DETECT ============== */
      if(data == &mic_sensor) {
        PRINTF("******* SOUND DETECTED *******\n");
#if ADC_ACQUISITION_ON
        SENSORS_MEASURE(mic_sensor);
#endif
#if WITH_IPSO
        ipso_microclap_detection();
#endif
      }
    } // if( ev == sensors_event )
  }  /* while (1) */

  PROCESS_END();
}

PROCESS_THREAD(ipso_objects_process, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  PRINTF("Starting IPSO objects process\n");

  /* Initialize the OMA LWM2M engine */
  lwm2m_engine_init();

  /* Register default LWM2M objects */
  lwm2m_engine_register_default_objects();

  /* Register default IPSO objects */
  ipso_objects_init();

  setup_lwm2m_servers();

  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
