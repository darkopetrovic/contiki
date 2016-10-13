/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Solar power (ina3221) resource
 * \author
 *      Darko Petrovic
 */


#include "contiki.h"
#include "custom-coap.h"

#include "ina3221-sensor.h"

static void res_init();
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_periodic_handler(void);

PERIODIC_RESOURCE(res_solar,
                  "title=\"Solar panel\";rt=\"solar-panel\";if=\"power\";ct=\"application/senml+json\";obs",
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
static uint16_t chan_val[2];

#if REST_DELAY_RES_START && APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
  if( !strncmp(p->name, CONFIG_PERIODIC_PARAM_NAME, strlen(p->name)) ){
    rest_update_resource_interval(&res_solar, p->value);
    return 0;
  }
  return 1;
}
#endif /* REST_DELAY_RES_START */

static void
res_init()
{
#if REST_DELAY_RES_START && APPS_APPCONFIG
  app_config_create_parameter(res_solar.url, CONFIG_PERIODIC_PARAM_NAME, "0", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	uint8_t input_current_int, input_voltage_int;
	uint16_t input_current_frac, input_voltage_frac;
	uint16_t ampere;

	COAP_BLOCKWISE_SETTINGS_LIST(res_solar);

	// get sensor value only once for the blockwise transfer
	if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){

		SENSORS_ACTIVATE(ina3221_sensor);
		SENSORS_MEASURE(ina3221_sensor);
		chan_val[0] = ina3221_sensor.value(INA3221_CH1_SHUNT_VOLTAGE);
		chan_val[1] = ina3221_sensor.value(INA3221_CH1_BUS_VOLTAGE);
		SENSORS_DEACTIVATE(ina3221_sensor);

		ampere = chan_val[0]/INA3221_SHUNT_RESISTOR_CH1;
		input_current_frac = ampere % 1000;
		input_current_int = ampere/1000;

		input_voltage_frac = chan_val[1] % 1000;
		input_voltage_int = chan_val[1]/1000;

		resource_add_message(res_solar.url, REST.type.APPLICATION_JSON,
				"{\"e\":["
					"{\"n\":\"solar_current\",\"v\":%d.%03de-3,\"u\":\"A\"},"
					"{\"n\":\"solar_voltage\",\"v\":%d.%03d,\"u\":\"V\"}"
				"],\"bn\":\"power\"}",
					input_current_int, input_current_frac,
					input_voltage_int, input_voltage_frac);
	}

	REST.set_header_max_age(response, res_solar.periodic->period / CLOCK_SECOND);
	COAP_BLOCKWISE_TRANSFER(res_solar);


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
    REST.notify_subscribers(&res_solar);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	COAP_UPDATE_SETTINGS(res_solar);
}

/** @} */
