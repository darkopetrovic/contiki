/**
 * \addtogroup coap-resources
 * @{
 *
 * \file
 *      Temperatures (sht21) resource
 * \author
 *      Darko Petrovic
 */

#include "contiki.h"
#include "custom-coap.h"
#if APP_CONFIG_STORAGE_COFFEE
#include "app-config.h"
#include <stdlib.h> /* strtol */
#endif

#include "ccs811-sensor.h"

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

PERIODIC_RESOURCE(res_air,
                  "title=\"Air quality\";rt=\"air\";if=\"sensor\";ct=\"application/senml+json\";obs",
                  res_init,
                  res_get_handler,
                  res_post_put_handler,
                  res_post_put_handler,
                  NULL,
                  5*CLOCK_SECOND,
                  res_periodic_handler);

#if REST_DELAY_RES_START && APPS_APPCONFIG
static uint8_t
callback(struct parameter *p)
{
#if APP_CONFIG_STORAGE_COFFEE
    static int fd;
    char buf[10];
#endif /* APP_CONFIG_STORAGE_COFFEE */

  if( !strncmp(p->name, CONFIG_PERIODIC_PARAM_NAME, strlen(p->name)) ){
    rest_update_resource_interval(&res_air, p->value);
    return 0;
  } else if( !strncmp(p->name, "state", strlen(p->name)) ){
    ccs811_sensor.configure(SENSORS_ACTIVE, p->value);

#if APP_CONFIG_STORAGE_COFFEE
      // set the baseline parameter when the sensor is turned on
      if(p->value && (fd = cfs_open("ccs811_baseline", CFS_READ))>0){
        cfs_seek(fd, 0, CFS_SEEK_SET);
        cfs_read(fd, buf, 10);
        cfs_close(fd);
        ccs811_sensor.configure(CCS811_CURRENT_BASELINE, (uint16_t)strtol(buf, NULL, 10));
      }
#endif /* APP_CONFIG_STORAGE_COFFEE */

    return 0;
  }
  return 1;
}
#endif /* REST_DELAY_RES_START */

static void
res_init()
{
#if REST_DELAY_RES_START && APPS_APPCONFIG
  app_config_create_parameter(res_air.url, CONFIG_PERIODIC_PARAM_NAME, "0", callback);
  app_config_create_parameter(res_air.url, "state", "0", callback);
#endif
}

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  uint32_t sensors_value[3];

  COAP_BLOCKWISE_SETTINGS_LIST(res_air);

  if( OBS_NOTIF_OR_FRST_BLCK_TRSF() ){
    SENSORS_MEASURE(ccs811_sensor);
    sensors_value[0] = ccs811_sensor.value(CCS811_SENSE_CO2);
    sensors_value[1] = ccs811_sensor.value(CCS811_SENSE_TVOC);
    sensors_value[2] = ccs811_sensor.status(CCS811_CURRENT_BASELINE);
    resource_add_message(res_air.url, REST.type.APPLICATION_JSON,
        "{\"e\":["
        "{\"n\":\"CO2\",\"v\":%d,\"u\":\"ppm\"},"
        "{\"n\":\"TVOC\",\"v\":%d,\"u\":\"ppm\"},"
        "{\"n\":\"Baseline\",\"v\":%d}"
        "]}",
        sensors_value[0], sensors_value[1], sensors_value[2]);
    }

  REST.set_header_max_age(response, res_air.periodic->period / CLOCK_SECOND);
  COAP_BLOCKWISE_TRANSFER(res_air);
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
    REST.notify_subscribers(&res_air);
  }
}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  COAP_UPDATE_SETTINGS(res_air);
}

/** @} */
