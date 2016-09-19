/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Sensor (sht21) resource
 * \author
 *      Darko Petrovic
 */

#include <string.h>
#include "contiki.h"
#include "custom-coap.h"

#include <stdlib.h>

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
static void res_periodic_handler(void);

PERIODIC_RESOURCE(res_humidity,
                  "title=\"Humidity\";rt=\"rel-humidity\";if=\"sensor\";ct=\"application/senml+json\";obs",
                  res_init,
                  res_get_handler,
                  res_post_put_handler,
                  res_post_put_handler,
                  NULL,
                  5*CLOCK_SECOND,
                  res_periodic_handler);

/*
 * Use local resource state that is accessed by res_get_handler() and altered by res_periodic_handler() or PUT or POST.
 */

#if REST_DELAY_RES_START && APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
  if( !strncmp(p->name, CONFIG_PERIODIC_PARAM_NAME, strlen(p->name)) ){
    rest_update_resource_interval(&res_humidity, p->value);
    return 0;
  }
  return 1;
}
#endif /* REST_DELAY_RES_START */

static void
res_init()
{
#if REST_DELAY_RES_START && APPS_APPCONFIG
  app_config_create_parameter(res_humidity.url, CONFIG_PERIODIC_PARAM_NAME, "0", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	uint8_t integral;
	uint8_t fractional;
	uint16_t sensors_value;

	COAP_BLOCKWISE_SETTINGS_LIST(res_humidity);

	if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){

		sensors_value = 5010;

		fractional = sensors_value % 100;
		integral = sensors_value / 100;

		resource_add_message(res_humidity.url, REST.type.APPLICATION_JSON,
				"{\"e\":[{\"n\":\"%s\",\"v\":%d.%d,\"u\":\"%%RH\"}]}",
					  res_humidity.url, integral, fractional);
	}

	REST.set_header_max_age(response, res_humidity.periodic->period / CLOCK_SECOND);
	COAP_BLOCKWISE_TRANSFER(res_humidity);
}
/*
 * Additionally, a handler function named [resource name]_handler must be implemented for each PERIODIC_RESOURCE.
 * It will be called by the REST manager process with the defined period.
 */
static void
res_periodic_handler()
{
  /* Usually a condition is defined under with subscribers are notified, e.g., large enough delta in sensor reading. */
  if(1) {
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_humidity);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	COAP_UPDATE_SETTINGS(res_humidity);
}

/** @} */
