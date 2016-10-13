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
 * \addtogroup coap-rest
 * @{
 *
 * \defgroup coap-resources CoAP resources
 * @{
 *
 * \file
 *  Header file for the smart settings library
 *
 * \author
 * Darko Petrovic
 *
 */
#ifndef _CUSTOM_COAP_H
#define _CUSTOM_COAP_H

#include "contiki-conf.h"
#include "rest-engine.h"
#include "er-coap.h"
#if APPS_APPCONFIG
#include "app-config.h"
#endif

/** The name of the resource the client can use to retrieve all configuration
 * from the devices. */
#define CONFIG_RESOURCE_NAME              "configuration"

/** The parameter used by the client to set the wake-up interval for each
 * periodic resource. */
#define CONFIG_PERIODIC_PARAM_NAME        "interval"

/** The maximum size of a resource message stored in memory.
 * Previously: 280 is not enough with /energest full statistics enabled. */
#define RESOURCE_REPRESENTATION_MAX_SIZE  400

/** The maximum stored resource representation.  */
#define MAX_RESOURCE_REPRESENTATION       6

#define MAX_PERIODIC_RESSOURCES           10

/** The time the resource message is available for the client during a blockwise transfer.
 * The system keep the RDC enabled if a message is not fully retrieved. Thus to prevent the
 * RDC to be enabled too long if the client don't GET for the message, this lifetime parameter
 * set when the message is cleared from the memory and thus enable again the deactivation
 * of the RDC. This parameter can be seen as the EXCHANGE_LIFETIME parameter mentionned
 * in draft-ietf-core-block-17 p.11 §2.4*/
#define RESOURCE_REPRESENTATION_LIFETIME  20    // in seconds

#define OBS_NOTIFICATION()                (offset == NULL)
#define OBS_NOTIF_OR_FRST_BLCK_TRSF()     (offset == NULL || (offset != NULL && *offset == 0))

/** Handle the blockwise transfer of the resource data. */
#define COAP_BLOCKWISE_TRANSFER(resource)   coap_blockwise_transfer(&resource, request, response, buffer, preferred_size, offset)

#if APPS_APPCONFIG
/** Call this function in the POST/PUT handler function of the resource. */
#define COAP_UPDATE_SETTINGS(resource)      coap_update_setting(&resource, request, response, buffer, preferred_size, offset)

/** Call this function in the GET handler function of the resource.
 * We exit the get handler if the client is requesting the settings (?p). */
#define COAP_BLOCKWISE_SETTINGS_LIST(resource); if(coap_blockwise_settings_list(&resource, request, response, buffer, preferred_size, offset)){return;}
#else /* APPS_APPCONFIG */
#define COAP_UPDATE_SETTINGS(resource)
#define COAP_BLOCKWISE_SETTINGS_LIST(resource);
#endif /* APPS_APPCONFIG */

/** As specified in draft-ietf-core-block-17 §2.4
 * We actually use only the URI :
 * [The server may identify the sequence by the combination of the requesting
 * end-point and the URI being the same in each block-wise request].*/
typedef struct resource_message_t {
  struct resource_message_t *next;        /* for LIST */
  const char *resource_url;
  char message[RESOURCE_REPRESENTATION_MAX_SIZE + 1];
  uint8_t etag[COAP_ETAG_LEN];
  uint8_t etag_len;
  struct ctimer lifetime;
  unsigned int content_format;
} resource_message_t;

void coap_blockwise_transfer(resource_t *resource, void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
uint8_t coap_blockwise_settings_list(resource_t *resource, void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
void coap_update_setting(resource_t *resource, void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
resource_message_t* resource_add_message(const char *resurl, unsigned int content_format, const char *format , ...);
resource_message_t* resource_get_freshmessage_by_resurl(const char *resurl);
resource_message_t* resource_get_oldestmessage_by_resurl(const char *resurl);
resource_message_t* resource_get_oldestmessage(void);
resource_message_t* resource_get_message_by_etag(const uint8_t *etag, int etag_len);
void resource_clear_message(resource_message_t *m);
uint8_t resource_pending_msg();
resource_message_t* resource_list_get_head(void);

#endif /* _CUSTOM_COAP_H */

/** @} */
/** @} */
