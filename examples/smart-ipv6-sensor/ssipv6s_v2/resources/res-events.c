/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Event based sensors (motion, micro)
 * \author
 *      Darko Petrovic
 */

#include "contiki.h"
#include "custom-coap.h"

#include "pir-sensor.h"
#include "mic-sensor.h"

/** \cond */
#define DEBUG 0
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

static void res_init();
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_events_handler();

EVENT_RESOURCE(res_events,
               "title=\"Events\";rt=\"events\";if=\"sensor\";ct=\"application/senml+json\";obs",
               res_init,
               res_get_handler,
               res_post_put_handler,
               res_post_put_handler,
               NULL,
               res_events_handler);

extern int32_t res_motion_counter;
extern uint8_t res_micro_clap_counter;
extern resource_t res_motion, res_micro;

static void
res_init()
{

}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  COAP_BLOCKWISE_SETTINGS_LIST(res_events);

   if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){
     resource_add_message(res_events.url, REST.type.APPLICATION_JSON,
             "{\"e\":["
             "{\"n\":\"%s\",\"v\":%lu,\"u\":\"count\"},"
             "{\"n\":\"%s\",\"v\":%lu,\"u\":\"count\"}"
             "]}",
             res_motion.url, res_motion_counter, res_micro.url, res_micro_clap_counter);
   }

   COAP_BLOCKWISE_TRANSFER(res_events);
}
/*
 * Additionally, a handler function named [resource name]_handler must be implemented for each PERIODIC_RESOURCE.
 * It will be called by the REST manager process with the defined period.
 */
static void
res_events_handler()
{
  /* Usually a condition is defined under with subscribers are notified, e.g., event was above a threshold. */
  if(1) {
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_events);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	COAP_UPDATE_SETTINGS(res_events);
}

/** @} */
