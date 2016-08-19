/**
 * \addtogroup smart-custom-rdc
 * @{
 *
 * \file
 *  Custom RDC Driver
 *
 *  \author
 *  Darko Petrovic
 */
#include "contiki.h"
#include "custom-rdc.h"

#if ENABLE_CUSTOM_RDC

#include "net/netstack.h"
#include "lpm.h"
#include "dev/sys-ctrl.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-icmp6.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if DEBUG
#include "dev/leds.h"
#endif

#define DIFF(a,b) ((a)-(b))

#define DEEP_SLEEP_PM2_THRESHOLD    100

#if ENERGEST_CONF_ON
static unsigned long irq_energest = 0;

#define ENERGEST_IRQ_SAVE(a) do { \
    a = energest_type_time(ENERGEST_TYPE_IRQ); } while(0)
#define ENERGEST_IRQ_RESTORE(a) do { \
    energest_type_set(ENERGEST_TYPE_IRQ, a); } while(0)
#else
#define ENERGEST_IRQ_SAVE(a) do {} while(0)
#define ENERGEST_IRQ_RESTORE(a) do {} while(0)
#endif

#if CRDC_COAP_IS_ENALBED
#define coap_confirmable_transaction_exist()  coap_confirmable_transaction_exist()
#define resource_pending_msg()                resource_pending_msg()
#else
#define coap_confirmable_transaction_exist()  0
#define resource_pending_msg()                0
#endif

#define DEEP_SLEEP_PM2_THRESHOLD    100

static struct ctimer ct_rdc;
static rtimer_clock_t finish_time;
static uint8_t rdc_is_on;

uint8_t crdc_battery_level;

// we restart the engine once the CRDC is initilized
extern struct process rest_engine_process;
extern struct process tcpip_process;


char *
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

static void
stop_rdc(void *ptr)
{
#if DEBUG && CRDC_COAP_IS_ENALBED
  if( coap_confirmable_transaction_exist() ){
    PRINTF("CRDC: > Confirmable notification found!\n");
  }

  if( resource_pending_msg() ){
    resource_message_t *m = NULL;
    PRINTF("CRDC: > Blockwise transfer not finished! Remaining representations:\n");
    for(m = resource_list_get_head(); m; m = m->next) {
      PRINTF("   - Resource:'/%s', ETag:%#018x, Timeout:%lu\n",
          m->resource_url, (unsigned int)*(uint32_t*)m->etag,
          timer_remaining(&(m->lifetime.etimer.timer)));
    }
  }
#endif

  /* Don't stop the RDC if:
   *  - the server is waiting a CoAP acknowledge (after confirmable message)
   *  - if a RS or NS message has been sent, and thus waiting on RA or NA
   *  - blockwise transfer is not finished, wait the client to get from complete message */
  if ( !coap_confirmable_transaction_exist() &&
      !resource_pending_msg() )
  {
      crdc_disable_rdc(0);
      PRINTF("CRDC: The RDC is DISABLED.\n");
#if DEBUG
      leds_off(LEDS_YELLOW);
#endif
  } else {
    // we prolong the rdc
    crdc_period_start( CRDC_DEFAULT_DURATION );
  }
}

void
crdc_init(void)
{
#if UIP_CONF_ROUTER
  rdc_is_on = 1;
#else /* UIP_CONF_ROUTER */
  rdc_is_on = 0;
  disableRDC(0);
#endif
}

void
crdc_lpm_enter(void)
{
  static clock_time_t next_expiration_old;
  clock_time_t next_expiration;
  rtimer_clock_t next_wakeup_time;
  uip_ds6_nbr_t *nbr;

  if( !rdc_is_on ){
    next_expiration = etimer_next_expiration_time();

    /* When 'next_expiration' is 0 that means there is no more etimer pending. And thus,
     * the SoC goes in sleep mode indefinetely until the button is pushed if configured
     * to power up the device.
     * */
    if( next_expiration ){
      if(next_expiration == next_expiration_old){
        return;
      }
      next_expiration_old = next_expiration;
      next_wakeup_time = (float)(next_expiration / CLOCK_SECOND) * RTIMER_SECOND;
      // schedule the next wake-up time -> we can go to sleep
      rtimer_arch_schedule( next_wakeup_time );
      SET_DEEP_SLEEP_MODE();
      if(next_wakeup_time <= RTIMER_NOW() ||
          next_wakeup_time-RTIMER_NOW() < DEEP_SLEEP_PM2_THRESHOLD){
        return;
      }

#if 0
      PRINTF("CRDC: Next wake-up in %s second(s) (next: %lu) (now: %lu) (clock: %lu).\n",
              float2str((float)(next_wakeup_time-RTIMER_NOW())/RTIMER_SECOND, 2),
              next_wakeup_time, RTIMER_NOW(), next_expiration);
#else
      PRINTF("CRDC: Next wake-up in %s second(s).\n",
                    float2str((float)(next_wakeup_time-RTIMER_NOW())/RTIMER_SECOND, 2));
#endif

    } else {
      PRINTF("CRDC: Sleep indefinitely...\n");
    }
#if DEBUG
    clock_delay_usec(5000); // to display correctly PRINTF before sleep
#endif

    ENTER_SLEEP_MODE();
    /* ZzZZzZZzZZZzzzZzzZZzzzzzzzZzZZzZzzzzzzzzzzzZZzZZZzzZZzZZZzzzZZzzzz
     * The wake-up can come from the rtimer or the gpio, nothing else.
     * ZzZZzZZzZZZzzzZzzZZzzzzzzzZzZZzZzzzzzzzzzzzZZzZZZzzZZzZZZzzzZZzzzz */

    /* Enable RDC for some seconds to receive RA in response to RS or ... */
    if(timer_remaining(&(uip_ds6_timer_rs.timer)) < CLOCK_SECOND){
      crdc_period_start(3);
    }
#if UIP_ND6_SEND_NA
    else {
      /* ... to receive NA in response to NS. */
      nbr = nbr_table_head(ds6_neighbors);
      while(nbr != NULL) {
#if CONF_6LOWPAN_ND
        if(nbr->state == NBR_TENTATIVE && stimer_remaining(&nbr->sendns) < CLOCK_SECOND)
#else /* CONF_6LOWPAN_ND */
        if((nbr->state == NBR_INCOMPLETE || nbr->state == NBR_PROBE)
            && stimer_remaining(&nbr->sendns) < CLOCK_SECOND)
#endif /* CONF_6LOWPAN_ND */
        {
          crdc_period_start(3);
          break;
        }
        nbr = nbr_table_next(ds6_neighbors, nbr);
      }
    }
#endif /* UIP_ND6_SEND_NA */

    PRINTF("CRDC:  *** RTimer time: %lu ***\n", RTIMER_NOW()/RTIMER_ARCH_SECOND);
  } else {
    lpm_enter();
  }
}

void
crdc_enable_rdc(void)
{
  // Accelerate DS periodic
#if DEBUG
  PROCESS_CONTEXT_BEGIN(&tcpip_process);
  etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND/10);
  PROCESS_CONTEXT_END(&tcpip_process);
#else
  PROCESS_CONTEXT_BEGIN(&tcpip_process);
  etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND/10);
  PROCESS_CONTEXT_END(&tcpip_process);
#endif
  enableRDC();
}

void
crdc_disable_rdc(uint8_t keep_radio)
{
  if(keep_radio){
#if DEBUG
    PROCESS_CONTEXT_BEGIN(&tcpip_process);
    etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND);
    PROCESS_CONTEXT_END(&tcpip_process);
#else
    PROCESS_CONTEXT_BEGIN(&tcpip_process);
    etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND/10);
    PROCESS_CONTEXT_END(&tcpip_process);
#endif
  } else {
    // Deccelerate DS periodic
    PROCESS_CONTEXT_BEGIN(&tcpip_process);
    etimer_set(&uip_ds6_timer_periodic, UIP_DS6_CONF_PERIOD);
    PROCESS_CONTEXT_END(&tcpip_process);
  }
  disableRDC(keep_radio);
}

void
crdc_clear_stop_rdc_timer(void)
{
  ctimer_stop(&ct_rdc);
}

void
crdc_period_start(uint32_t seconds)
{
  /* Normally, the rtimer interrupt should be enabled in
   * rtimer_set() -> rtimer_arch_schedule() but rtimer_set()
   * doesn't call any more rtimer_arch_schedule() because the 'next_rtimer'
   * is no more NULL since it has run before and when we disable the RDC before
   * the rtimer ISR is triggered 'next_rtimer' is not set to NULL.
   */

#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
  /* When usb cable is plugged in, the radio is constantly on. */
  if(NODE_TYPE_ROUTER || USB_IS_PLUGGED()){
    return;
  }
#endif /* UIP_CONF_DYN_HOST_ROUTER*/

  /* Prevent RDC to start if battery level is critical. */
  if( crdc_battery_level == BATLEV_CRITICAL ){
    return;
  }

  /* Reduce the RDC period to save a little energy if battery voltage is low. */
  if( crdc_battery_level == BATLEV_LOW && seconds > RDC_SAFE_PERIOD ){
    seconds = RDC_SAFE_PERIOD;
  }

  // need to turn-on the RDC to send a packet
  if( !seconds && !rdc_is_on){
    enableRDC();
    ctimer_set(&ct_rdc, 8, stop_rdc, NULL);
    PRINTF("CRDC: The RDC is ENABLED to send a packet.\n");
    return;
  }

  if( !rdc_is_on ){
    finish_time = RTIMER_NOW() + seconds*RTIMER_SECOND;
    crdc_enable_rdc();
    ctimer_set(&ct_rdc, CLOCK_SECOND * seconds, stop_rdc, NULL);
    PRINTF("CRDC: The RDC is ENABLED (for %lu seconds).\n", seconds);
  } else if( seconds && rdc_is_on && (RTIMER_NOW() + seconds*RTIMER_SECOND) > finish_time ) {
    // prolong the actual RDC
    finish_time = RTIMER_NOW() + seconds*RTIMER_SECOND;
    ctimer_set(&ct_rdc, CLOCK_SECOND * seconds, stop_rdc, NULL);
    PRINTF("CRDC: The RDC is PROLONGED (for %lu seconds).\n", seconds);
  }
}

#endif // ENABLE_CUSTOM_RDC

/** @} */
