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
 *          Project specific configuration.
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* ********************************************************************** */
/* Application configuration                                              */
/* ********************************************************************** */

#define REST_MAX_CHUNK_SIZE                 128

/** Delay the start of resources.  */
#define REST_CONF_DELAY_RES_START           1

/** Activate energy consumption module */
#define ENERGEST_CONF_ON                    1

/** Activate the Communication power statistics */
#define CONTIKIMAC_CONF_COMPOWER            1

/* ********************************************************************** */
/* RPL configuration                                                      */
/* ********************************************************************** */

/**
 * \sixlowpanndrpl 	We disable the probing for the host when
 *                  6lowpan-nd is enabled. 
 *
 * \todo implement dynamic host/router for the probing
 */
#if CONF_6LOWPAN_ND && UIP_CONF_IPV6_RPL && !UIP_CONF_ROUTER
#define RPL_CONF_WITH_PROBING               0
#endif         
/** 
 * \needreview The DAO message couldn't be received sometimes by the node. 
 * Need to do more tests.
 */
#define RPL_CONF_WITH_DAO_ACK               1

/* ********************************************************************** */
/* 6LoWPAN-ND configuration                                               */
/* ********************************************************************** */

#define UIP_CONF_ND6_REGISTRATION_LIFETIME  5

/* ********************************************************************** */
/* System configuration                                                   */
/* ********************************************************************** */

#define UIP_CONF_TCP                        0

#define	WATCHDOG_CONF_ENABLE                0

//#define UIP_CONF_ROUTER   1
//#define RDC_CONF_SLEEPING_HOST 0

/**
 * The USB is initiliazed only when the USB cable is plugged in. Therefore,
 * there is no extra current consumption on the battery due to the USB process.
 */
#define CC2538_CONF_QUIET                   0

/**
 * Useful to print debug message on the terminal while debuging USB features
 * (initilization, plug, unplug, command input, ...). */
#define DEBUG_USB_WITH_UART                 0

/** Enable USB (commands and print) */
#define DBG_CONF_USB                        0

#define UIP_CONF_BUFFER_SIZE                400

/** Reduce the maximum amount of concurrent UDP connections (default 10). */
#define UIP_CONF_UDP_CONNS                  5

/** Maximum routes to store */
#define UIP_CONF_MAX_ROUTES                 2

/* Expected reassembly requirements   */
#define SICSLOWPAN_CONF_FRAGMENT_BUFFERS    4

/** Maximum neighbors to store in the Neighbors Table */
#if CONTIKI_TARGET_CC2538DK || CONTIKI_TARGET_SSIPV6S_V1 || CONTIKI_TARGET_SSIPV6S_V2
#if CONF_6LOWPAN_ND && !UIP_CONF_ROUTER
#define NBR_TABLE_CONF_MAX_NEIGHBORS        UIP_CONF_MAX_ROUTES
#else /* CONF_6LOWPAN_ND && !UIP_CONF_ROUTER */
#define NBR_TABLE_CONF_MAX_NEIGHBORS        3
#endif /* CONF_6LOWPAN_ND && !UIP_CONF_ROUTER */
#endif

#if !UIP_CONF_ROUTER && RDC_CONF_SLEEPING_HOST
#define UIP_DS6_CONF_PERIOD                 CLOCK_SECOND*10
#endif

/* ********************************************************************** */
/* Platform specific configurations.                                      */
/* ********************************************************************** */

/* Even if CONTIKI_TARGET_CC2538DK is set with the ssipv6s platform in the
 * board.h file, this configuration file is included before. */
#if CONTIKI_TARGET_CC2538DK || CONTIKI_TARGET_SSIPV6S_V1 || CONTIKI_TARGET_SSIPV6S_V2
#define CC2538_RF_CONF_CHANNEL              25
#define LPM_CONF_MAX_PM                     2
#define LPM_CONF_ENABLE                     1
/* CFS Configuration */
#define APP_CONFIG_CONF_STORAGE_COFFEE      1
#define FLASH_CONF_FW_ADDR                  CC2538_DEV_FLASH_ADDR
#define COFFEE_CONF_START                   0x230000  // 96th page
#define COFFEE_CONF_SIZE                    262144
#define COFFEE_CONF_APPEND_ONLY             0
#define COFFEE_CONF_MICRO_LOGS              1
#define COFFEE_CONF_NAME_LENGTH             60
#endif

#if CONTIKI_TARGET_Z1
#define ENTER_SLEEP_MODE()  _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF)
#endif

/* ********************************************************************** */
/* Automatic (re)definition when required.                                */
/* ********************************************************************** */

#if DEBUG_USB_WITH_UART
#define USB_SERIAL_CONF_ENABLE              1
#undef CC2538_CONF_QUIET
#define CC2538_CONF_QUIET                   0
#undef DBG_CONF_USB
#define DBG_CONF_USB                        0
#endif

#if DBG_CONF_USB
/* If we set the QUIET mode, the global configuration will unset USB_SERIAL_CONF_ENABLE
 * and the USB will be unusable (USB_SERIAL_CONF_ENABLE depends on DBG_CONF_USB) */
#undef CC2538_CONF_QUIET
#define CC2538_CONF_QUIET                   0
/* Prevent the initilisation of the standard UART when we use USB. */
#define UART_CONF_ENABLE                    0
/* Avoid spurious current consumption at device startup if UART is not in use. */
#define STARTUP_CONF_VERBOSE                0
#endif

#endif /* PROJECT_CONF_H_ */
