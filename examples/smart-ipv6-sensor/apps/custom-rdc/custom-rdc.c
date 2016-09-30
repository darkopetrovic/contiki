/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 */
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

#if RDC_SLEEPING_HOST

#include "net/netstack.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-icmp6.h"

#ifdef CONTIKI_TARGET_CC2538DK
#include "lpm.h"
#include "dev/sys-ctrl.h"
#define DEEP_SLEEP_PM2_THRESHOLD    100
#endif /* CONTIKI_TARGET_CC2538DK */

#if UIP_CONF_IPV6_RPL
#include "net/rpl/rpl-private.h"
#endif

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if DEBUG
#include "dev/leds.h"
#endif

#if CRDC_COAP_IS_ENALBED
#include "custom-coap.h"
#include "er-coap-transactions.h"
#else
#define coap_confirmable_transaction_exist()  0
#define resource_pending_msg()                0
#endif

#define UIP_ICMP_BUF ((struct uip_icmp_hdr *)&uip_buf[UIP_LLIPH_LEN + uip_ext_len])

static struct ctimer ct_rdc;
static rtimer_clock_t finish_time;
static uint8_t rdc_is_on;

// we restart the engine once the CRDC is initilized
extern struct process rest_engine_process;
extern struct process tcpip_process;

#if ENERGEST_CONF_ON && CONTIKI_TARGET_CC2538DK
static unsigned long irq_energest;
#endif /* CONTIKI_TARGET_CC2538DK */

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

static 
void stop_rdc(void *ptr)
{
#if UIP_ND6_SEND_NA
  uip_ds6_nbr_t *nbr;
  uint8_t ns_msg_is_sent;
#endif /* UIP_ND6_SEND_NA */

/**
 * Small security: with the dynamic host/router feature, if the CRDC period was
 * enabled while the node is a host, this stop_rdc function is scheduled. 
 * If the node becomes a router meanwhile, we prevent to stop the RDC. 
 * Same for the USB cable.
 */
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
  if(NODE_TYPE_ROUTER || USB_IS_PLUGGED()){
    return;
  }
#endif /* UIP_CONF_DYN_HOST_ROUTER*/

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
#endif /* DEBUG && CRDC_COAP_IS_ENALBED */

  /* Check whether the node has sent an NS message to register
   * to a router (6lowpan-nd) or resolve address (ndp). */
#if UIP_ND6_SEND_NA
  ns_msg_is_sent = 0;
  nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
#if CONF_6LOWPAN_ND
    if(nbr->state == NBR_TENTATIVE && nbr->nscount)
#else /* CONF_6LOWPAN_ND */
    if((nbr->state == NBR_INCOMPLETE || nbr->state == NBR_PROBE) && nbr->nscount)
#endif /* CONF_6LOWPAN_ND */
    {
      ns_msg_is_sent = 1;
      break;
    }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }

#endif /* UIP_ND6_SEND_NA */

  /* Don't stop the RDC if:
   *  - the server is waiting a CoAP acknowledge (after confirmable message)
   *  - if a RS or NS message has been sent, and thus waiting on RA or NA
   *  - blockwise transfer is not finished, wait the client to get from complete message */
  if ( !coap_confirmable_transaction_exist() && !resource_pending_msg() 
#if UIP_ND6_SEND_NA
      && !ns_msg_is_sent
#endif
      )
  {
      crdc_disable_rdc(0);
      PRINTF("CRDC: The RDC is DISABLED.\n");
  } else {
    // we prolong the rdc
    crdc_period_start( CRDC_DEFAULT_DURATION );
  }
}

void
crdc_init(void)
{
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
  if(NODE_TYPE_ROUTER){
    rdc_is_on = 1;
  } else {
    rdc_is_on = 0;
    crdc_disable_rdc(0);
  }
#else 
  rdc_is_on = 0;
  crdc_disable_rdc(0);
#endif
}

void
crdc_lpm_enter(void)
{
  static clock_time_t next_expiration_old;
  clock_time_t next_expiration;
  rtimer_clock_t next_wakeup_time;
  //uip_ds6_nbr_t *nbr;

  if(!rdc_is_on){

    // do we need this with cc2538?
    //nvic_interrupt_unpend(NVIC_INT_SM_TIMER);
    next_expiration = etimer_next_expiration_time();

    /* When 'next_expiration' is 0 that means there is no more etimer pending. And thus,
     * the SoC goes in sleep mode indefinetely until the button is pushed if configured
     * to power up the device.
     * */
    if( next_expiration ){
      if(next_expiration == next_expiration_old){
        //return;
      }
      next_expiration_old = next_expiration;
      next_wakeup_time = (float)(next_expiration / CLOCK_SECOND) * RTIMER_SECOND;

#ifdef CONTIKI_TARGET_CC2538DK
      REG(SYS_CTRL_PMCTL) = LPM_CONF_MAX_PM;
      /* Check whether the next wake-up time is effectively planned in
       * the future and if it's not too small. */
      if(next_wakeup_time <= RTIMER_NOW() ||
          next_wakeup_time-RTIMER_NOW() < DEEP_SLEEP_PM2_THRESHOLD){
        next_wakeup_time = RTIMER_NOW() + RTIMER_SECOND/NETSTACK_RDC_CHANNEL_CHECK_RATE;
      }
#endif /* CONTIKI_TARGET_CC2538DK */

      // schedule the next wake-up time -> we can go to sleep
      rtimer_arch_schedule( next_wakeup_time );

#if 0
      PRINTF("CRDC: Next wake-up in %s second(s) (next: %X) (now: %u) (clock: %lu).\n",
              float2str((float)(next_expiration-clock_time())/CLOCK_SECOND, 2),
              next_wakeup_time, RTIMER_NOW(), next_expiration);
#else
      PRINTF("CRDC: Next wake-up in %s second(s).\n",
                    float2str((float)(next_expiration-clock_time())/CLOCK_SECOND, 2));
#endif

    } else {
      PRINTF("CRDC: Sleep indefinitely...\n");
    }
#if CONTIKI_TARGET_CC2538DK
#if DEBUG
    clock_delay_usec(5000); // to display correctly PRINTF before sleep
#endif
#if ENERGEST_CONF_ON
    ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);
    energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
#endif /* ENERGEST_CONF_ON */
#endif /* CONTIKI_TARGET_CC2538DK */

    ENTER_SLEEP_MODE();
    /* ZzZZzZZzZZZzzzZzzZZzzzzzzzZzZZzZzzzzzzzzzzzZZzZZZzzZZzZZZzzzZZzzzz
     * The wake-up can come from the rtimer or the gpio, nothing else.
     * ZzZZzZZzZZZzzzZzzZZzzzzzzzZzZZzZzzzzzzzzzzzZZzZZZzzZZzZZZzzzZZzzzz */

#if ENERGEST_CONF_ON && CONTIKI_TARGET_CC2538DK
    irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
    ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON && CONTIKI_TARGET_CC2538DK */



#if 0
    /* Enable RDC for some seconds to receive RA in response to RS or ... */
    if(timer_remaining(&(uip_ds6_timer_rs.timer)) < CLOCK_SECOND){
      PRINTF("CRDC: Sending RS...\n");
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
          PRINTF("CRDC: Sending NS...\n");
          crdc_period_start(3);
          break;
        }
        nbr = nbr_table_next(ds6_neighbors, nbr);
      }
    }
#endif /* UIP_ND6_SEND_NA */

#if UIP_CONF_IPV6_RPL
    if(rpl_get_any_dag() == NULL && rpl_get_next_dis() >= RPL_DIS_INTERVAL-1) {
      PRINTF("CRDC: Sending DIS...\n");
      crdc_period_start(3);
    } 
#endif

#endif

    PRINTF("CRDC: *** Time: %lu *** \n", clock_time());
  } else {
    /* RDC is enabled -> normal behaviour. */
#if CONTIKI_TARGET_CC2538DK
    lpm_enter();
#else
    ENTER_SLEEP_MODE();
#endif
  }
}

void
crdc_enable_rdc(void)
{
  // Accelerate DS periodic
#if DEBUG
  PROCESS_CONTEXT_BEGIN(&tcpip_process);
  etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND);
  PROCESS_CONTEXT_END(&tcpip_process);
#else
  //PROCESS_CONTEXT_BEGIN(&tcpip_process);
  //etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND/10);
  //PROCESS_CONTEXT_END(&tcpip_process);
  PROCESS_CONTEXT_BEGIN(&tcpip_process);
  etimer_reset(&uip_ds6_timer_periodic);
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
    //PROCESS_CONTEXT_BEGIN(&tcpip_process);
    //etimer_set(&uip_ds6_timer_periodic, CLOCK_SECOND/10);
    //PROCESS_CONTEXT_END(&tcpip_process);
#endif
  } else {
    // Deccelerate DS periodic
    //PROCESS_CONTEXT_BEGIN(&tcpip_process);
    //etimer_reset_with_new_interval(&uip_ds6_timer_periodic, UIP_DS6_PERIOD);
    //PROCESS_CONTEXT_END(&tcpip_process);
  }
  disableRDC(keep_radio);
}

void
crdc_clear_stop_rdc_timer(void)
{
  ctimer_stop(&ct_rdc);
}

static uint8_t
expect_response(void)
{
#if UIP_CONF_IPV6_RPL
  if( (UIP_ICMP_BUF->type == ICMP6_RS || UIP_ICMP_BUF->type == ICMP6_NS)
      || (UIP_ICMP_BUF->type == ICMP6_RPL && UIP_ICMP_BUF->icode == RPL_CODE_DIS)
      || (UIP_ICMP_BUF->type == ICMP6_RPL && UIP_ICMP_BUF->icode == RPL_CODE_SEC_DIS)
#if RPL_WITH_DAO_ACK
      || (UIP_ICMP_BUF->type == ICMP6_RPL && UIP_ICMP_BUF->icode == RPL_CODE_DAO)
      || (UIP_ICMP_BUF->type == ICMP6_RPL && UIP_ICMP_BUF->icode == RPL_CODE_SEC_DAO)
#endif
    )
#else /* UIP_CONF_IPV6_RPL */
  if(UIP_ICMP_BUF->type == ICMP6_RS || UIP_ICMP_BUF->type == ICMP6_NS)
#endif /* UIP_CONF_IPV6_RPL */
  {
    return 1;
  } else {
    return 0;
  }
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
  /* When usb cable is plugged in, the RDC is constantly on. */
  if(NODE_TYPE_ROUTER || USB_IS_PLUGGED()){
    return;
  }
#endif /* UIP_CONF_DYN_HOST_ROUTER*/

  /* Turn-on the RDC to send a packet. Enable it for CRDC_WAIT_RESPONSE
   * if a response is expected. */
  if( !seconds && !rdc_is_on){
    enableRDC();
    if(expect_response()){
      ctimer_set(&ct_rdc, CLOCK_SECOND * CRDC_WAIT_RESPONSE, stop_rdc, NULL);
      PRINTF("CRDC: The RDC is ENABLED (for %d seconds) to receive a response.\n", 
        CRDC_WAIT_RESPONSE);
    } else {
      ctimer_set(&ct_rdc, 8, stop_rdc, NULL);
      PRINTF("CRDC: The RDC is ENABLED to send a packet.\n");
    }
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
