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
 * \addtogroup cc2538-examples
 * @{
 *
 * \file
 * Project specific configuration
 */
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#define CONF_6LOWPAN_ND               1
//#define USB_SERIAL_CONF_ENABLE        0
//#undef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER               0
//#define UIP_CONF_DYN_HOST_ROUTER      0

/** Enable the external 32k oscillator */
#define SYS_CTRL_CONF_OSC32K_USE_XTAL         1

//#define NETSTACK_CONF_RDC     nullrdc_driver

#define UIP_CONF_ND6_SEND_NA              1
#define UIP_CONF_IPV6_RPL                         0


#define CC2538_RF_CONF_CHANNEL              25
#define LPM_CONF_MAX_PM                   2
#define UIP_DS6_CONF_PERIOD               CLOCK_SECOND*10

#define UIP_CONF_TCP				0

#define	WATCHDOG_CONF_ENABLE		0

#endif /* PROJECT_CONF_H_ */

/** @} */
