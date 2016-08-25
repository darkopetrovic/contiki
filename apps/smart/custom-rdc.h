/**
 * \addtogroup smart-sensor
 * @{
 *
 * \defgroup smart-custom-rdc Custom RDC Driver
 * @{
 *
 * \file
 *  Header file for the Custom RDC Driver
 *
 * \author
 * Darko Petrovic
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOMRDC_H
#define __CUSTOMRDC_H

#include "rtimer-arch.h"

#ifndef CRDC_CONF_ENABLE_CUSTOM_RDC
#define ENABLE_CUSTOM_RDC             1 // enabled by default
#else
#define ENABLE_CUSTOM_RDC             CRDC_CONF_ENABLE_CUSTOM_RDC
#endif

#ifndef ENTER_SLEEP_MODE
#define ENTER_SLEEP_MODE()            do { asm("wfi"::); } while(0)
#endif

#ifndef CRDC_CONF_DEFAULT_DURATION
#define CRDC_DEFAULT_DURATION         1 // in seconds
#else
#define CRDC_DEFAULT_DURATION         CRDC_CONF_DEFAULT_DURATION
#endif

/* The custom rdc looks if there is pending transations to not stop
 * the RDC if this is the case.
 * */
#ifndef CRDC_CONF_COAP_IS_ENALBED
#define CRDC_COAP_IS_ENALBED          0
#else
#define CRDC_COAP_IS_ENALBED          CRDC_CONF_COAP_IS_ENALBED
#endif

/* The duration in seconds the system keeps the RDC on to receive
 * a packet in response. */
#ifndef CRDC_CONF_WAIT_RESPONSE
#define CRDC_WAIT_RESPONSE          3 // second(s)
#else
#define CRDC_WAIT_RESPONSE          CRDC_CONF_WAIT_RESPONSE
#endif

#ifndef USB_IS_PLUGGED
#define USB_IS_PLUGGED()              0
#endif

extern void rtimer_isr(void);

#define enableRDC()       rdc_is_on = 1;\
                          NETSTACK_RDC.on();\

/* When turned off, ContikiMAC doesn't kill automatically the next wake-up.
 * Therefore we disable the incoming wake-up from ContikiMAC with rtimer_arch_schedule(0).
 * Furthermore, we call rtimer_isr() to properly finish the LPL cycle. */
#define disableRDC(x)     NETSTACK_RDC.off(x);\
                          rtimer_arch_schedule(0);\
                          rtimer_run_next();\
                          rdc_is_on = 0
/**
 * \brief   Initialize the custom RDC system.
 *
 */
void crdc_init();

/**
 * \brief   Enable the RDC for a short period of time.
 *
 * This function is used for the custom RDC. When the SoC wake-up it enables
 * temporarily the RDC in order to receive messages properly. This is especially
 * needed for the CoAP Observer system: sometimes the SoC send confirmable CoAP
 * notification in order to verify that the subscriber is still interest in the
 * resource (draft-ietf-core-observe-16 / section 1.2). If we don't enable the RDC,
 * the Acknowledgment can be missed and then the CoAP server remove the client
 * from the list.
 *
 * This function is equally used when the user press the button on the device in order
 * to enable the RDC for a longer period of time (to receive configuration, etc ...).
 *
 * \param int Number of seconds
 */
void crdc_period_start(uint32_t seconds);

void crdc_lpm_enter();
uint8_t crdc_get_rdc_status();
void crdc_enable_rdc();
void crdc_disable_rdc(uint8_t keep_radio);
void crdc_clear_stop_rdc_timer(void);

extern uint8_t crdc_battery_level;

#endif /* __CUSTOMRDC_H */

/** @} */
/** @} */
