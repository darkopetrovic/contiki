/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Motion detector resource
 * \author
 *      Darko Petrovic
 */


#include "contiki.h"
#include "custom-coap.h"

#include "dev/mic-sensor.h"

/** \cond */
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif
/** \endcond */

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

static void res_init();
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_micro_handler();

#define DEFAULT_MD_SLEEP_DURATION	120	// seconds

EVENT_RESOURCE(res_micro,
               "title=\"Microphone\";rt=\"micro\";if=\"sensor\";ct=\"text/plain\";obs",
               res_init,
               res_get_handler,
               res_post_put_handler,
               res_post_put_handler,
               NULL,
               res_micro_handler);

uint8_t res_micro_clap_counter;

#if APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
  if( !strncmp(p->name, "state", strlen(p->name)) ){
    mic_sensor.configure(SENSORS_ACTIVE, p->value);
    return 0;
  }
  return 1;
}
#endif

static void
res_init()
{
#if APPS_APPCONFIG
  app_config_create_parameter(res_micro.url, "state", "0", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

#if ADC_ACQUISITION_ON
  size_t strpos = 0;            /* position in overall string (which is larger than the buffer) */
  size_t bufpos = 0;            /* position within buffer (bytes written) */
  size_t tmplen = 0;
  char txtsetting[10];
  uint16_t i;
  int16_t *adcvalues;

  adcvalues = (int16_t *)mic_sensor.value(0);

  for(i=0;i<ADC_SAMPLES;i++){
    snprintf(txtsetting, 10, "%03X", (adcvalues[i]>>4)&0x0FFF);
    ADD_STRING_IF_POSSIBLE(txtsetting, >);
  }

  coap_set_payload(response, buffer, bufpos);
  coap_set_header_content_format(response, TEXT_PLAIN);

  if(i >= ADC_SAMPLES) {
    PRINTF("RES: (micro) DONE\n");
    *offset = -1;
  } else {
    PRINTF("RES: (micro) MORE\n");
    *offset += preferred_size;
  }
#else /* ADC_ACQUISITION_ON */
  COAP_BLOCKWISE_SETTINGS_LIST(res_micro);

  if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){
    resource_add_message(res_micro.url, REST.type.APPLICATION_JSON,
            "{\"e\":[{\"n\":\"%s\",\"v\":%lu,\"u\":\"count\"}]}",
            "claps", res_micro_clap_counter);
  }

  COAP_BLOCKWISE_TRANSFER(res_micro);
#endif /* ADC_ACQUISITION_ON */
}
/*
 * Additionally, res_event_handler must be implemented for each EVENT_RESOURCE.
 * It is called through <res_name>.trigger(), usually from the server process.
 */
static void
res_micro_handler()
{
  /* Usually a condition is defined under with subscribers are notified, e.g., event was above a threshold. */
  if(1) {
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_micro);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  COAP_UPDATE_SETTINGS(res_micro);
}

/** @} */
