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

/**
 * \file
 *          Proect specific configuration.
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/**
 * \sixlowpanndrpl 	We disable the probing for the host when
 *                  6lowpan-nd is enabled. 
 *
 * \todo implement dynamic host/router for the probing
 */
#if CONF_6LOWPAN_ND && UIP_CONF_IPV6_RPL && !UIP_CONF_ROUTER
#define RPL_CONF_WITH_PROBING               0
#endif         

/* RPL configuration */

/** 
 * \needreview The DAO message couldn't be received sometimes by the node. 
 * Need to do more tests.
 */
#define RPL_CONF_WITH_DAO_ACK               1

/* 6LoWPAN-ND configuration */

#define UIP_CONF_ND6_REGISTRATION_LIFETIME  5

/* System configuration */

#define UIP_CONF_TCP                        0

#define	WATCHDOG_CONF_ENABLE                0

#if !UIP_CONF_ROUTER && CRDC_CONF_ENABLE_CUSTOM_RDC
#define UIP_DS6_CONF_PERIOD                 CLOCK_SECOND*10
#endif

/* Platform specific configurations. */

#if CONTIKI_TARGET_CC2538DK
#define CC2538_RF_CONF_CHANNEL              25
#define LPM_CONF_MAX_PM                     2
#endif

#if CONTIKI_TARGET_Z1
#define ENTER_SLEEP_MODE()  _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF)
#endif

#endif /* PROJECT_CONF_H_ */
