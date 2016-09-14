/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Temperatures (sht21 & tmp100) resource
 * \author
 *      Darko Petrovic
 */

#include "contiki.h"
#include "custom-coap.h"

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

static void res_init();
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_periodic_handler(void);

PERIODIC_RESOURCE(res_temperature,
                  "title=\"Temperature\";rt=\"temperature\";if=\"sensor\";ct=\"application/senml+json\";obs",
                  res_init,
                  res_get_handler,
                  res_post_put_handler,
                  res_post_put_handler,
                  NULL,
                  5*CLOCK_SECOND,
                  res_periodic_handler);

#if REST_DELAY_RES_START && APP_CONFIG
static uint8_t
callback(struct parameter *p)
{
  if( !strncmp(p->name, SETTINGS_PERIODIC_PARAM_NAME, strlen(p->name)) ){
    rest_update_resource_interval(&res_temperature, p->value);
    return 0;
  }
  return 1;
}
#endif /* REST_DELAY_RES_START */

static void
res_init()
{
#if REST_DELAY_RES_START && APP_CONFIG
  app_config_create_parameter(res_temperature.url, SETTINGS_PERIODIC_PARAM_NAME, "5", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	int8_t temp1_integral;
	uint8_t temp1_fractional;

	int8_t temp2_integral;
	uint8_t temp2_fractional;

	uint16_t sensors_value[2];

	COAP_BLOCKWISE_SETTINGS_LIST(res_temperature);

	if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){

		sensors_value[0] = 2500;
		sensors_value[1] = 2520;

		temp1_fractional = sensors_value[0] % 100;
		temp1_integral = sensors_value[0] / 100;

		temp2_fractional = sensors_value[1] % 100;
		temp2_integral = sensors_value[1] / 100;

		resource_add_message(res_temperature.url, REST.type.APPLICATION_JSON,
				"{\"e\":["
				"{\"n\":\"temp.front\",\"v\":%d.%02d,\"u\":\"Cel\"},"
				"{\"n\":\"temp.back\",\"v\":%d.%02d,\"u\":\"Cel\"}"
				"]}",
					  temp1_integral, temp1_fractional,
					  temp2_integral, temp2_fractional);
	}

	REST.set_header_max_age(response, res_temperature.periodic->period / CLOCK_SECOND);
	COAP_BLOCKWISE_TRANSFER(res_temperature);
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
    REST.notify_subscribers(&res_temperature);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	COAP_UPDATE_SETTINGS(res_temperature);
}

/** @} */
