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

#include "dev/pir-sensor.h"

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
static void res_motion_handler();

#define DEFAULT_MD_SLEEP_DURATION	120	// seconds

EVENT_RESOURCE(res_motion,
               "title=\"Motion detection\";rt=\"motion\";if=\"sensor\";ct=\"application/senml+json\";obs",
               res_init,
               res_get_handler,
               res_post_put_handler,
               res_post_put_handler,
               NULL,
               res_motion_handler);

static int32_t event_counter = 0;

#if APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
	if( !strncmp(p->name, "state", strlen(p->name)) ){
		pir_sensor.configure(SENSORS_ACTIVE, p->value);
		return 0;
	} else if ( !strncmp(p->name, "sleep-duration", strlen(p->name)) ) {
		pir_sensor.configure(PIR_DEACTIVATION_DELAY, p->value);
		return 0;
	}
	return 1;
}
#endif

static void
res_init()
{
#if APPS_APPCONFIG
  app_config_create_parameter(res_motion.url, "state", "0", callback);
  /** \todo use DEFAULT_MD_SLEEP_DURATION */
  app_config_create_parameter(res_motion.url, "sleep-duration", "120", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

	COAP_BLOCKWISE_SETTINGS_LIST(res_motion);

	if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){
		resource_add_message(res_motion.url, REST.type.APPLICATION_JSON,
						"{\"e\":[{\"n\":\"%s\",\"v\":%lu,\"u\":\"count\"}]}",
						res_motion.url, event_counter);
	}

	COAP_BLOCKWISE_TRANSFER(res_motion);

}
/*
 * Additionally, res_event_handler must be implemented for each EVENT_RESOURCE.
 * It is called through <res_name>.trigger(), usually from the server process.
 */
static void
res_motion_handler()
{
  /* Do the update triggered by the event here, e.g., sampling a sensor. */
  ++event_counter;

  /* Usually a condition is defined under with subscribers are notified, e.g., event was above a threshold. */
  if(1) {
    PRINTF("TICK %lu for /%s\n", event_counter, res_motion.url);
    //blink_led(LEDS_YELLOW, CLOCK_SECOND/4, 1);
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_motion);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	COAP_UPDATE_SETTINGS(res_motion);
}

/** @} */
