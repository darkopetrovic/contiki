/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
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
 * \file
 *    
 * \author 
 * 			Darko Petrovic <darko.petrovic.85@gmail.com>
 *
 */

#include "contiki.h"
#include "dev/leds.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if PLATFORM_HAS_LEDS

static struct ctimer ct_blink;
static unsigned char led_to_blink;
static uint32_t period_of_blink;
static unsigned char leds_status;

PROCESS(blink_process, "Smart Blink");

static void
stop_blink(void *ptr)
{
	process_exit(&blink_process);
	PRINTF("Blink process STOPPED.\n");
}

void 
blink_leds(unsigned char l, uint32_t period, uint8_t duration)
{
	// stop the previous blink
	process_exit(&blink_process);

  /* If duration is 0 we blink indefinitely until stop function is called. */
  if(duration){
    ctimer_set(&ct_blink, CLOCK_SECOND * duration, stop_blink, NULL);
  } 
	
	leds_status = leds_get();
	led_to_blink = l;
	period_of_blink = period;

	//process_exit(&blink_process);
	process_start(&blink_process, NULL);
	PRINTF("Blink process STARTED (leds: %2X, period: %lu, duration: %d).\n", l, period, duration);
}

void
blink_leds_stop(void)
{
  stop_blink(NULL);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{

  static struct etimer et;

  PROCESS_EXITHANDLER(goto exit;);

  PROCESS_BEGIN();

  etimer_set(&et, period_of_blink/2);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    leds_toggle( led_to_blink );
    etimer_set(&et, period_of_blink/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    leds_toggle( led_to_blink );
    etimer_reset(&et);
  }

 exit:
  //leds_off( led_to_blink );
  leds_set(leds_status);
  PROCESS_END();
}
#endif /* PLATFORM_HAS_LEDS */
