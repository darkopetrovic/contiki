/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Sensor configuration resource
 * \author
 *      Darko Petrovic
 */

#include "contiki.h"
#include "custom-coap.h"

#include <stdlib.h>
#include <string.h>
#include "lib/list.h"
#include "apps/er-coap/er-coap-observe.h"


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

static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

/*
 * A handler function named [resource name]_handler must be implemented for each RESOURCE.
 * A buffer for the response payload is provided through the buffer pointer. Simple resources can ignore
 * preferred_size and offset, but must respect the REST_MAX_CHUNK_SIZE limit for the buffer.
 * If a smaller block size is requested for CoAP, the REST framework automatically splits the data.
 */
RESOURCE(res_config,
         "title=\"Configuration\";rt=\"config\";ct=\"application/senml+json\"",
         NULL,
         res_get_handler,
         res_post_put_handler,
         res_post_put_handler,
         NULL);

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	COAP_BLOCKWISE_SETTINGS_LIST(res_config);
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	size_t len = 0;
	const char *strvalue = NULL;
	uint32_t intval;
	coap_observer_t *obs = NULL;

#if REST_DELAY_RES_START
	periodic_resource_t *periodic_resource = NULL;
#endif /* REST_DELAY_RES_START */

	if( (len = REST.get_post_variable(request, "clear-observers", &strvalue)) )
	{
		intval = atoi(strvalue);
		if(intval == 1){
			for(obs = coap_get_list_observers(); obs; obs = obs->next)
			{
				coap_remove_observer(obs);
			}
#if REST_DELAY_RES_START
			for(periodic_resource = list_head(rest_get_periodic_resources());
				periodic_resource; periodic_resource = periodic_resource->next)
			{
				if( periodic_resource->period )
				{
				  rest_update_resource_interval((resource_t *)periodic_resource->resource, 0);
				}
			}
#endif /* REST_DELAY_RES_START */
			REST.set_response_status(response, REST.status.CHANGED);
		}

	}

	else if( (len = REST.get_post_variable(request, "radio-channel", &strvalue)) )
	{
		intval = atoi(strvalue);
		/* myTODO: change radio channel wirelessly */
		/* Timeout the change of the radio channel. */
	}

	else {
		COAP_UPDATE_SETTINGS(res_config);
	}

}

/** @} */
