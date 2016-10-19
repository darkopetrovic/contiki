/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Power sensor (ina3221) resource
 * \author
 *      Darko Petrovic
 */

#include "contiki.h"
#include "custom-coap.h"

#include "ina3221-sensor.h"

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
/** \endcond */

#include <stdlib.h>
#include "dev/sys-ctrl.h"

static void res_init();
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_periodic_handler(void);

PERIODIC_RESOURCE(res_power,
                  "title=\"Power values\";rt=\"solar-battery\";if=\"power\";ct=\"application/senml+json\";obs",
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
static uint16_t chan_val[4];

#if REST_DELAY_RES_START && APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
  if( !strncmp(p->name, CONFIG_PERIODIC_PARAM_NAME, strlen(p->name)) ){
    rest_update_resource_interval(&res_power, p->value);
    return 0;
  }
  return 1;
}
#endif /* REST_DELAY_RES_START */

static void
res_init()
{
#if REST_DELAY_RES_START && APPS_APPCONFIG
  app_config_create_parameter(res_power.url, CONFIG_PERIODIC_PARAM_NAME, "0", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  uint8_t input_voltage_int;
  uint16_t input_voltage_frac;

  double input_current_frac, input_current_int;

  uint8_t output_voltage_int;
  uint16_t output_voltage_frac;

  double output_current_frac, output_current_int;

  float ampere;
  uint8_t battery_soc;

  COAP_BLOCKWISE_SETTINGS_LIST(res_power);

  // get sensor value only once (first block) for the blockwise transfer
  if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){
    /* Sleep for 125ms before the measurement to let the battery voltage relax after the voltage drop.
     * With this delay the difference with the relaxed battery level is within 0.05 to 0.08V. */
    deep_sleep_ms(125, NO_GPIO_INTERRUPT, 0);

    SENSORS_ACTIVATE(ina3221_sensor);
    SENSORS_MEASURE(ina3221_sensor);
    chan_val[0] = ina3221_sensor.value(INA3221_CH1_SHUNT_VOLTAGE);
    chan_val[1] = ina3221_sensor.value(INA3221_CH1_BUS_VOLTAGE);
    chan_val[2] = ina3221_sensor.value(INA3221_CH2_SHUNT_VOLTAGE);
    chan_val[3] = ina3221_sensor.value(INA3221_CH2_BUS_VOLTAGE);
    SENSORS_DEACTIVATE(ina3221_sensor);

    // solar panel current & voltage
    ampere = (float)(chan_val[0]/1e2/INA3221_SHUNT_RESISTOR_CH1);	// in mA
    input_current_frac = modf(ampere, &input_current_int);
    // IN- is measured, we compute the IN+ bus voltage by adding the shunt voltage
    input_voltage_frac = (chan_val[1]+(uint8_t)(chan_val[0]/1e2)) % 1000;
    input_voltage_int = (chan_val[1]+(uint8_t)(chan_val[0]/1e2)) / 1000;

    // battery current & voltage
    /* Battery current (or rather load current is intentionally dismissed here
     * for the end user because the resolution on the channel 2 is 267uA
     * and is varying to much). */
    ampere = (float)(chan_val[2]/1e2/INA3221_SHUNT_RESISTOR_CH2);	// in mA
    output_current_frac = modf(ampere, &output_current_int);
    output_voltage_frac = chan_val[3] % 1000;
    output_voltage_int = chan_val[3] / 1000;

    if( chan_val[3] >= BATTERY_NOM_VOLTAGE*1000 ){
      battery_soc = 100;
    } else {
      battery_soc = 100-(BATTERY_NOM_VOLTAGE-(float)(chan_val[3]/1000.0)) *
          (100.0/(BATTERY_NOM_VOLTAGE-BATTERY_CUT_VOLTAGE));
    }

    resource_add_message(res_power.url, REST.type.APPLICATION_JSON,
        "{\"e\":["
          "{\"n\":\"solar_current\",\"v\":%d.%03de-3,\"u\":\"A\"},"
          "{\"n\":\"solar_voltage\",\"v\":%d.%03d,\"u\":\"V\"},"
          "{\"n\":\"battery_voltage\",\"v\":%d.%03d,\"u\":\"V\"},"
          "{\"n\":\"battery_current\",\"v\":%d.%03de-3,\"u\":\"A\"},"
          "{\"n\":\"battery_soc\",\"v\":%d,\"u\":\"%%\"}"
        "],\"bn\":\"power\"}",
          (uint32_t)input_current_int, (uint32_t)(input_current_frac*1e3),
          input_voltage_int, input_voltage_frac,
          output_voltage_int, output_voltage_frac,
          (uint32_t)output_current_int, (uint32_t)(output_current_frac*1e3),
          battery_soc);

  }

  REST.set_header_max_age(response, res_power.periodic->period / CLOCK_SECOND);

  COAP_BLOCKWISE_TRANSFER(res_power);

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
    REST.notify_subscribers(&res_power);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  COAP_UPDATE_SETTINGS(res_power);
}

/** @} */
