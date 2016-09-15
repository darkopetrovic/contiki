/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Resource for the Energest module
 * \author
 *      Darko Petrovic
 */

#include "contiki.h"
#include "custom-coap.h"
#include "power-track.h"

#include "sys/energest.h"

#if CONTIKIMAC_CONF_COMPOWER
#include "sys/compower.h"
#endif

#include <math.h>

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

#define REPR_TIME	1e3	// in miliseconds
/** \endcond */

#if FLASH_STORAGE_ON
#include "eeprom-emul.h"
#endif

#include <stdlib.h>
#include "dev/sys-ctrl.h"

/** Provide every energest duration in the /energest representation. */
#define	STATISTICAL_DATA		1

static void res_init();
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_periodic_handler(void);

PERIODIC_RESOURCE(res_energest,
                  "title=\"Energest\";rt=\"energest-module\";if=\"energest\";ct=\"application/text\";obs",
                  res_init,
                  res_get_handler,
                  res_post_put_handler,
                  res_post_put_handler,
                  NULL,
                  60*CLOCK_SECOND,
                  res_periodic_handler);

/*
 * Use local resource state that is accessed by res_get_handler() and altered by res_periodic_handler() or PUT or POST.
 */

#if REST_DELAY_RES_START && APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
  if( !strncmp(p->name, SETTINGS_PERIODIC_PARAM_NAME, strlen(p->name)) ){
    rest_update_resource_interval(&res_energest, p->value);
    return 0;
  }
  return 1;
}
#endif /* REST_DELAY_RES_START */

static void
res_init()
{
#if REST_DELAY_RES_START && APPS_APPCONFIG
  app_config_create_parameter(res_energest.url, SETTINGS_PERIODIC_PARAM_NAME, "0", callback);
#endif
}
static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

	double charge_consumed_fractpart, charge_consumed_intpart;
	/*double remaining_charge_fractpart, remaining_charge_intpart;*/

	COAP_BLOCKWISE_SETTINGS_LIST(res_energest);

	// get sensor value only once (first block) for the blockwise transfer
	if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){

		charge_consumed_fractpart = modf(energest_data.charge_consumed, &charge_consumed_intpart);
		/*remaining_charge_fractpart = modf(energest_data.remaining_charge, &remaining_charge_intpart);*/

		resource_add_message(res_energest.url, REST.type.APPLICATION_JSON,
			"{\"e\":["
				"{\"n\":\"duration\",\"v\":%lue-3,\"u\":\"s\"},"
				"{\"n\":\"q_consumed\",\"v\":%d.%03d,\"u\":\"C\"},"
				/*"{\"n\":\"Q_remain\",\"v\":%d.%03d,\"u\":\"C\"},"*/
				"{\"n\":\"avg_current\",\"v\":%lue-6,\"u\":\"A\"}"
				/*"{\"n\":\"Est. Lifetime\",\"v\":%lu,\"u\":\"s\"}"*/
#if STATISTICAL_DATA
				",{\"n\":\"raw_duration\",\"sv\":\"%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
								"%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu"
#if CONTIKIMAC_CONF_COMPOWER
       ",%lu,%lu,%lu,%lu"
#endif /* CONTIKIMAC_CONF_COMPOWER */
		    "\"}"
#endif /* STATISTICAL_DATA */
			"],\"bn\":\"energest\"}",
			(uint32_t)((float)energest_data.all_time/RTIMER_SECOND*REPR_TIME),
			(uint32_t)charge_consumed_intpart, (uint32_t)(charge_consumed_fractpart*1e3),
			/*(uint32_t)remaining_charge_intpart, (uint32_t)(remaining_charge_fractpart*1e3),*/
			energest_data.avg_current
			/*,energest_data.estimated_lifetime*/
#if STATISTICAL_DATA
			,(uint32_t)((float)clock_time()/CLOCK_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.all_cpu/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.all_lpm/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.all_transmit/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.all_listen/RTIMER_SECOND*REPR_TIME),
#if CONTIKIMAC_CONF_COMPOWER
			 (uint32_t)((float)energest_data.all_idle_transmit/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.all_idle_listen/RTIMER_SECOND*REPR_TIME),
#endif
			 (uint32_t)((float)energest_data.all_flash_read/RTIMER_SECOND*REPR_TIME*REPR_TIME),
			 (uint32_t)((float)energest_data.all_flash_write/RTIMER_SECOND*REPR_TIME*REPR_TIME),
			 (uint32_t)((float)energest_data.all_flash_erase/RTIMER_SECOND*REPR_TIME*REPR_TIME),
			 (uint32_t)((float)energest_data.all_sensors_ina3221/RTIMER_SECOND*REPR_TIME*REPR_TIME),
			 (uint32_t)((float)energest_data.all_sensors_sht21/RTIMER_SECOND*REPR_TIME*REPR_TIME),
			 (uint32_t)((float)energest_data.all_sensors_tmp100/RTIMER_SECOND*REPR_TIME*REPR_TIME),
			 (uint32_t)((float)energest_data.all_sensors_pir/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.all_leds/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.cpu/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.lpm/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.transmit/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.listen/RTIMER_SECOND*REPR_TIME)
#if CONTIKIMAC_CONF_COMPOWER
			 ,(uint32_t)((float)energest_data.idle_transmit/RTIMER_SECOND*REPR_TIME),
			 (uint32_t)((float)energest_data.idle_listen/RTIMER_SECOND*REPR_TIME)
#endif /* CONTIKIMAC_CONF_COMPOWER */
#endif /* STATISTICAL_DATA */
		);

	}

	REST.set_header_max_age(response, res_energest.periodic->period / CLOCK_SECOND);

	COAP_BLOCKWISE_TRANSFER(res_energest);

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
    REST.notify_subscribers(&res_energest);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	size_t len = 0;
	const char *strvalue = NULL;
	uint32_t intval;
	if( (len = REST.get_post_variable(request, "reset", &strvalue)) )
	{
		intval = atoi(strvalue);
		if(intval == 1){
		  powertrack_reset();
			REST.set_response_status(response, REST.status.CHANGED);
		}

	} else {
		COAP_UPDATE_SETTINGS(res_energest);
	}

}

/** @} */
