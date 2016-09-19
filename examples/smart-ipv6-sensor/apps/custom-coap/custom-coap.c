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
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *  Application level message implementation for resources
 *
 *  \author
 *  Darko Petrovic
 */
#include "contiki.h"
#include "custom-coap.h"

#if APPS_APPCONFIG
#include "app-config.h"
#endif

#include <stdarg.h> /* vsprintf */
#include <stdlib.h> /* strtol */

/** \cond */
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"


#define ADD_CHAR_IF_POSSIBLE(char) \
  if(strpos >= *offset && bufpos < preferred_size) { \
    buffer[bufpos++] = char; \
  } \
  ++strpos

#define ADD_STRING_IF_POSSIBLE(string, op) \
  tmplen = strlen(string); \
  if(strpos + tmplen > *offset) { \
    bufpos += snprintf((char *)buffer + bufpos, \
                       preferred_size - bufpos + 1, \
                       "%s", \
                       string \
                       + (*offset - (int32_t)strpos > 0 ? \
                          *offset - (int32_t)strpos : 0)); \
    if(bufpos op preferred_size) { \
      break; \
    } \
  } \
  strpos += tmplen

#define ADD_STRING_IF_POSSIBLE_NOLOOP(string) \
  tmplen = strlen(string); \
  if(strpos + tmplen > *offset) { \
    bufpos += snprintf((char *)buffer + bufpos, \
                       preferred_size - bufpos + 1, \
                       "%s", \
                       string \
                       + (*offset - (int32_t)strpos > 0 ? \
                          *offset - (int32_t)strpos : 0)); \
  } \
  strpos += tmplen
/** \endcond */

/* The system can store MAX_RESOURCE_REPRESENTATION messages independently of the resource.
 * This is not used by the .well-know/core and CONFIG_RESOURCE_NAME resources which
 * create the representation dynamically.
 */
MEMB(resource_message_memb, resource_message_t, MAX_RESOURCE_REPRESENTATION);
LIST(resource_message_list);

static uint8_t res_repr_counter;

static void
array_reverse(uint8_t *src, uint8_t *dst, uint8_t size)
{
  uint8_t i;
  for (i = 0; i < size; i++) {
    *(dst+i) = *(src+size-1-i);
  }
}

static void
clear_resource_message(void *ptr)
{
  resource_message_t *m = ptr;
  PRINTF("RES: Timeout\n");
  resource_clear_message(m);
}

/* Add a cached version of a representation for an ongoing sequence of requests */
resource_message_t *
resource_add_message(const char *resurl, unsigned int content_format, const char *format , ...)
{
  va_list arglist;
  uint32_t rtnow;
  resource_message_t *resmsg;

  /* An ETag value is generated for each message.
   * This feature is necessary for the consistency of the communication and to avoid message overlap.
   * For example, if the client was requesting the last block of a representation just after
   * a new notification (this can happen if the notificaiton interval is very short ~4-5 secs),
   * the server will provide the last block of this NEW message and clear this latter. Then the client
   * ask for the first block of this new representation but this latter doesn't exist anymore. This will
   * generate a BlockOutOfScope error and unnecessary retransmission by the client.
   *
   * The number of resource representation is limited to MAX_RESOURCE_REPRESENTATION.
   * (no matter the resource, can be of the same resource).
   *
   * A timeout of RESOURCE_REPRESENTATION_LIFETIME is started after which the server clear the
   * message to free space.
   *
   **/

  PRINTF("RES: Available space in cache %d/%d.\n", res_repr_counter, MAX_RESOURCE_REPRESENTATION );
  if( ++res_repr_counter > MAX_RESOURCE_REPRESENTATION ){
    PRINTF("RES: (warning) No more space in message cache. Clearing oldest.\n");
    resmsg = resource_get_oldestmessage();
    resource_clear_message(resmsg);
  }

  resource_message_t *m = memb_alloc(&resource_message_memb);

  if(m) {
    va_start( arglist, format );
    vsprintf( m->message, format, arglist );
    va_end( arglist );
    m->resource_url = resurl;
    m->content_format = content_format;
    /* Create an unique ETag for the representation.  */
    rtnow = RTIMER_NOW();
    memcpy(m->etag, &rtnow, 4);
    m->etag_len = 4;
    list_add(resource_message_list, m); /* list itself makes sure same element is not added twice */
    PRINTF("RES: Adding message (%d/%d) from resource '%s' with ETag %#018x.\n",
        res_repr_counter, MAX_RESOURCE_REPRESENTATION,
        m->resource_url, (unsigned int)*(uint32_t*)m->etag);

    // start a timer to remove the message after its lifetime expires
    ctimer_set(&m->lifetime, CLOCK_SECOND*RESOURCE_REPRESENTATION_LIFETIME, clear_resource_message, m);
  }
  return m;
}

void
resource_clear_message(resource_message_t *m)
{
  if(m) {
    PRINTF("RES: Message cleared for resource '%s' with ETag:%#018x.\n",
          m->resource_url, (unsigned int)*(uint32_t*)m->etag);
    ctimer_stop(&m->lifetime);
    list_remove(resource_message_list, m);
    memb_free(&resource_message_memb, m);
    res_repr_counter--;
  }
}

resource_message_t*
resource_get_freshmessage_by_resurl(const char *resurl)
{
  resource_message_t *iter = NULL;
  resource_message_t *fresh = NULL;
  /* new message are added at the end of the list */
  PRINTF("RES: Looking for freshest message for resource '%s'.\n", resurl);
  for(iter = (resource_message_t *)list_head(resource_message_list); iter; iter = iter->next) {
    if( !strcmp(resurl, iter->resource_url) ) {
      fresh = iter;
    }
  }
  if( fresh != NULL){
    PRINTF("RES: Found freshest message for resource '%s'.\n", fresh->resource_url);
  } else {
    PRINTF("RES: Fresh message not found.\n");
  }
  return fresh;
}

resource_message_t*
resource_get_oldestmessage_by_resurl(const char *resurl)
{
  resource_message_t *iter = NULL;
  /* new message are added at the end of the list */
  PRINTF("RES: Looking for oldest message for resource '%s'.\n", resurl);
  for(iter = (resource_message_t *)list_head(resource_message_list); iter; iter = iter->next) {
    if( !strcmp(resurl, iter->resource_url) ) {
      PRINTF("RES: Found oldest message for resource '%s'.\n", iter->resource_url);
      return iter;
    }
  }
  PRINTF("RES: (error) Oldest message not found.\n");
  return NULL;
}

resource_message_t*
resource_get_oldestmessage(void)
{
  resource_message_t *m = NULL;
  PRINTF("RES: Looking for oldest message.\n");
  m = (resource_message_t *)list_head(resource_message_list);
  if(m){
    return m;
  }
  PRINTF("RES: (error) Oldest message not found. Cache is empty.\n");
  return NULL;
}

resource_message_t*
resource_get_message_by_etag(const uint8_t *etag, int etag_len)
{
  resource_message_t *m = NULL;
  PRINTF("RES: Looking for message with Etag %#018x.\n", (unsigned int)*(uint32_t*)etag);
  for(m = (resource_message_t *)list_head(resource_message_list); m; m = m->next) {
    if( !memcmp(etag, (uint8_t *)m->etag, etag_len) ) {
      PRINTF("RES: Found stored message for Etag %#018x.\n", (unsigned int)*(uint32_t*)m->etag);
      return m;
    }
  }
  PRINTF("RES: (error) ETag message not found.\n");
  return NULL;
}

uint8_t
resource_pending_msg()
{
  return list_length(resource_message_list);
}

resource_message_t*
resource_list_get_head(void)
{
  return list_head(resource_message_list);
}

void
coap_blockwise_transfer(resource_t *resource, void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset)
{
  int16_t nbchars;
  int32_t _offset;
  resource_message_t *resmsg;
  int etag_len;
  uint8_t *etag;
  uint8_t etag_reversed[COAP_ETAG_LEN];

  /* Retrieve the temporarely stored CoAP message of the resource.
   * For the first block, the system calculate the actual value of the sensor.
   * But for subsequent blocks request by the client, the device must be able to provide
   * the rest of the SAME message. We use for this purpose the ETag option in the message.
   * This later is then cleared from the memory once it is completly retrieved by the client.
   *
   * Note that the client doesn't have to use the conditional request If-Match to do so. The device
   * check the ETag option in the request and found the resource representation for this ETag.
   *
   * */
  etag_len = coap_get_header_etag(request, (const uint8_t **)&etag);
  array_reverse(etag, etag_reversed, etag_len);

  /* If ETag option is not present in the request, we send the freshest representation of the resoure */
  if( !etag_len ){
    resmsg = resource_get_freshmessage_by_resurl( resource->url );
  } else {
    resmsg = resource_get_message_by_etag( etag_reversed, etag_len );
  }

  if( resmsg == NULL ){
    return;
  }

  /* (original comment of Contiki-OS)
  * For minimal complexity, request query and options should be ignored for GET on observable resources.
  * Otherwise the requests must be stored with the observer list and passed by REST.notify_subscribers().
  * This would be a TODO in the corresponding files in contiki/apps/erbium/!
  */
  REST.set_header_content_type(response, resmsg->content_format);

  /* Add to the response the entity-tag associated to the message. The client must
   * add this ETag in the request to retrieve thereafter this message.
   * ETag array is reversed because CoAP engine places LSB first the value in the packet and
   * get printed inversed when the message is parsed. The ETag is generated at the creation
   * of the representation and stored in memory with the message payload.
   * */
  array_reverse(resmsg->etag, etag_reversed, resmsg->etag_len);
  coap_set_header_etag(response, (const uint8_t *)etag_reversed, resmsg->etag_len);

  /* When Observe option is used, the coap observer engine set the pointer to NULL. */
  if( offset == NULL ){
    _offset = 0;
    /* During observer notification the system doesn't set automatically the MORE option in the message.
     * We do it here simply by controlling that the message cannot be transfered in one block with the size
     * requested by the client.
     *
     * myFEATURE: For all notification, the system use by default REST_MAX_CHUNK_SIZE for
     * the message size. The block2 size option at client registration isn't stored by Contiki and therefore
     * cannot be used for subsequent notification. We do not touch the actual implementation of Contiki
     * and let it be as it is.  */
    if( strlen(resmsg->message) > preferred_size ){

      // set MORE option
      coap_set_header_block2(response, 0, 1, preferred_size);
    }
  } else {
    /* offset for the next blocks of the blockwise transfer */
    _offset = *offset;
  }

  nbchars = snprintf((char *)buffer, preferred_size + 1, "%s", resmsg->message + _offset);

  /* snprintf() returns the total length of the string even if not stored in the buffer.
   * strpos is then used for the last block transfer for the remaining data. */
  if(nbchars > preferred_size) {
     nbchars = preferred_size;
  }

  REST.set_response_payload(response, buffer, nbchars);

  /* */
  if( offset != NULL ){
    /* IMPORTANT for chunk-wise: Signal chunk awareness to REST engine. */
    *offset += nbchars;

    /* End of blockwise transfer */
    if(*offset >= strlen(resmsg->message)) {
      *offset = -1;
      resource_clear_message(resmsg);
    }
  } else {
    /* Observer notification sent in one block transfer ... */
    if( strlen(resmsg->message) <= preferred_size ){
      // ... we remove directly the representation
      resource_clear_message(resmsg);
    }
  }

  /* The REST.subscription_handler() will be called for observable resources by the REST framework. */
}

#if APPS_APPCONFIG
void
coap_update_setting(resource_t *resource, void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

  struct parameter* parameter;
  size_t len = 0;
  const char *strvalue = NULL;
  int error = 0;
  uint32_t intval;
  char cntxt[20];

  memset(cntxt, 0, sizeof(cntxt));
  if(!strcmp(resource->url, CONFIG_RESOURCE_NAME)){
    strncpy(cntxt, APP_CONFIG_GENERAL, strlen(APP_CONFIG_GENERAL));
  } else {
    sprintf(cntxt, "%s", resource->url);
  }

  PRINTF("Trying to update parameter in '%s'.\n", cntxt);

  for(parameter = app_config_parameters_list_head(); parameter != NULL; parameter = list_item_next(parameter))
  {
    if( !strcmp(cntxt, parameter->context) )
    {
      if( (len = REST.get_post_variable(request, parameter->name, &strvalue)) )
      {
        if((intval=strtol(strvalue, NULL, 10)) != 0 || !strncmp(strvalue, "0", 1)){
          error = app_config_edit_parameter(parameter->context, parameter->name, NULL, intval);
        } else {
          error = app_config_edit_parameter(parameter->context, parameter->name, strvalue, 0);
        }
        // parameter was found don't need to look further
        break;
      } else {
        error = 2;
      }
    }
  }

  if( !error ) {
    REST.set_response_status(response, REST.status.CHANGED);
  } else if( error == 1 ) {
    REST.set_response_status(response, REST.status.FORBIDDEN);
  } else if( error == 2 ){
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  }

}

#if 1
uint8_t
coap_blockwise_settings_list(resource_t *resource, void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  size_t strpos = 0;            /* position in overall string (which is larger than the buffer) */
  size_t bufpos = 0;            /* position within buffer (bytes written) */
  size_t tmplen = 0;
  size_t urllen = 0;
  struct parameter* parameter;
  char txtsetting[100];
  uint8_t count_settings = 0;
  const char *param = NULL;

  static uint8_t is_setting_resource;
  uint8_t base_name = 0;

  if( OBS_NOTIFICATION() ){
    return 0;
  }

  /* Doing a GET on the 'CONFIG_RESOURCE_NAME' resource fetches every
   * existing parameters on the system. */
  is_setting_resource = !strcmp(resource->url, CONFIG_RESOURCE_NAME);
  tmplen = REST.get_query_variable(request, "p", &param);

  /* For GET request without '?p' on other resource than the global settings resource,
   * we exit this function and let the rest of the resource GET handler to finish.
   */
  if( !is_setting_resource && param==NULL ){
    return 0;
  }

  PRINTF("RES: Entering blockwise settings list for %s.\n", resource->url);

  urllen = strlen(resource->url);

  // request a particular parameter
  if( param != NULL && strncmp(param, "all", tmplen) ){
    // the query param string doesn't end with '\0'
    snprintf(txtsetting, tmplen+1, "%s", param);
    /* myFEATURE: we may improve the lookup of parameters by creating
     * special contexts like 'system, global, user, ...' and use search for
     * these parameters with for example ?p=system */

    parameter = app_config_parameter_lookup( resource->url, txtsetting );
    PRINTF("RES: Looking for setting context '%s' with key '%s' (length=%d)\n", resource->url, txtsetting, tmplen);
    if( parameter != NULL ){
      tmplen = snprintf((char*)buffer, tmplen, "%lu", parameter->value);
      PRINTF("RES: Setting %s, length=%d\n", buffer, tmplen);
      coap_set_header_content_format(response, TEXT_PLAIN);
      coap_set_payload(response, buffer, tmplen);
    } else {
      PRINTF("RES: Setting not found.\n");
      coap_set_status_code(response, NOT_FOUND_4_04);
      // set payload otherwise the response code NOT FOUND above doesn't work
      coap_set_payload(response, "", 1);
    }
    return 1;
  }


  if( *offset == 0 ){
    bufpos += snprintf((char *)buffer, preferred_size + 1, "{\"e\":[");
  }
  strpos += strlen("{\"e\":[");

  for(parameter = app_config_parameters_list_head(); parameter != NULL; parameter = list_item_next(parameter))
  {
    /* We don't know the total length of the message, so
     * we create the message dynamically as for the .well-know resource.
     *
     * By using strncmp instead of strcmp we get parameters for sub-resources equally.*/
    if( !strncmp(resource->url, parameter->context, urllen) || is_setting_resource )
    {
      count_settings++;

      if( parameter->context != NULL ){
        if( is_setting_resource ){
          // display full parameter path
          if(parameter->is_string){
            snprintf(txtsetting, 100,
               "{\"n\":\"%s?p=%s\",\"sv\":\"%s\"}", parameter->context, parameter->name,
               (char*)app_config_get_parameter_value(parameter->context, parameter->name));
          } else {
            snprintf(txtsetting, 100,
              "{\"n\":\"%s?p=%s\",\"v\":%lu}", parameter->context, parameter->name, parameter->value);
          }
        } else {
          // remove resource url base name
          if(parameter->is_string){
            snprintf(txtsetting, 100,
                    "{\"n\":\"%s?p=%s\",\"sv\":\"%s\"}", parameter->context+urllen,
                    parameter->name,
                    (char*)app_config_get_parameter_value(parameter->context, parameter->name));
          } else {
            snprintf(txtsetting, 100,
                    "{\"n\":\"%s?p=%s\",\"v\":%lu}", parameter->context+urllen,
                    parameter->name, parameter->value);
          }

          base_name = 1;
          PRINTF("RES: (settings list) Remove base name from '%s' -> '%s'\n",
              parameter->context, parameter->context+urllen);
        }

      } else {

        if(parameter->is_string){
          snprintf(txtsetting, 100,
             "{\"n\":\"?p=%s\",\"sv\":\"%s\"}", parameter->name,
             (char*)app_config_get_parameter_value(parameter->context, parameter->name));
        } else {
          snprintf(txtsetting, 100,
             "{\"n\":\"?p=%s\",\"v\":%lu}", parameter->name, parameter->value);
        }
      }

      ADD_STRING_IF_POSSIBLE(txtsetting, >);
      ADD_CHAR_IF_POSSIBLE(',');
    }
  }


  /* We can get here a second time if the block size is small. */
  if(parameter == NULL){
    // we remove the last ','
    if(strpos >= *offset && bufpos < preferred_size){
      bufpos--;
    }
    strpos--;

    ADD_CHAR_IF_POSSIBLE(']');

    if( base_name ){
      snprintf(txtsetting, 100, ",\"bn\":\"%s\"", resource->url);
      ADD_STRING_IF_POSSIBLE_NOLOOP(txtsetting);
    }

    ADD_CHAR_IF_POSSIBLE('}');
  }

  // overwrite the response if there is no settings
  if( count_settings == 0 ){
    coap_set_header_content_format(response, TEXT_PLAIN);
    coap_set_payload(response, "No Settings", 11);
  }

  else if(bufpos > 0) {
    PRINTF("RES: (settings list) BUF %d: %.*s\n", bufpos, bufpos, (char *)buffer);
    coap_set_payload(response, buffer, bufpos);
    coap_set_header_content_format(response, APPLICATION_JSON);
  }

  else if(strpos > 0) {
    coap_set_status_code(response, BAD_OPTION_4_02);
    coap_set_payload(response, "BlockOutOfScope", 15);
  }


  /* Detect the end of the response.
   * 'setting' is NULL when no 'break' has occured which means that we get through all resources.
   * The end of the response string ends in any case with the '}' charachter. But this character can
   * occurs within the response ...
   * myTODO: problem may occurs if the text block ends with '}'
   * */
  if(parameter == NULL && buffer[bufpos-1] == '}') {
    PRINTF("RES: (settings list) DONE\n");
    *offset = -1;
  } else {
    PRINTF("RES: (settings list) MORE at %s (%p)\n", parameter->name, parameter);
    *offset += preferred_size;
  }
  return 1;
}
#endif
#endif /* APPS_APPCONFIG */

/** @} */
