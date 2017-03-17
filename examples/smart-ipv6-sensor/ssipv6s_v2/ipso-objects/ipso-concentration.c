/*
 * Copyright (c) 2015, Yanzi Networks AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \addtogroup ipso-objects
 * @{
 */

/**
 * \file
 *         Implementation of OMA LWM2M / IPSO Concentration
 * \author
 *         Darko Petrovic
 */

#include <stdint.h>
#include "ipso-objects.h"
#include "ccs811-sensor.h"

#include "app-config.h"
#if APP_CONFIG_STORAGE_COFFEE
#include <stdlib.h> /* strtol */
#endif

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define IPSO_CO2_MIN      400
#define IPSO_CO2_MAX      5000

#define IPSO_TVOC_MIN     0
#define IPSO_TVOC_MAX     1000

#define CO2 0
#define TVOC 1

static int32_t min_co2_value;
static int32_t max_co2_value;
static int32_t min_tvoc_value;
static int32_t max_tvoc_value;

uint16_t co2_value;
uint16_t tvoc_value;
static int read_sensor_value(int32_t *value, uint8_t type);
static void handle_periodic_timer(void *ptr);
/*---------------------------------------------------------------------------*/
static int
sensor_co2_value(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  if(read_sensor_value(&value, CO2)) {
    return ctx->writer->write_int(ctx, outbuf, outsize, value);
  }
  return 0;
}

static int
sensor_tvoc_value(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  if(read_sensor_value(&value, TVOC)) {
    return ctx->writer->write_int(ctx, outbuf, outsize, value);
  }
  return 0;
}

static int
read_power_state(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int real_power_status = ccs811_sensor.status(CCS811_POWER_STATE);
  return ctx->writer->write_int(ctx, outbuf, outsize, real_power_status);
}

static int
write_power_state(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
#if APP_CONFIG_STORAGE_COFFEE
    static int fd;
    char buf[10];
    uint16_t baseline;
#endif /* APP_CONFIG_STORAGE_COFFEE */
  int32_t value;
  size_t len;
  len = ctx->reader->read_int(ctx, inbuf, insize, &value);

  ccs811_sensor.configure(SENSORS_ACTIVE, value);

#if APP_CONFIG_STORAGE_COFFEE
  if(value){
    // when turned on, read the baseline from memory if present
    // and configure the sensor with the value
    if(value && (fd = cfs_open("ccs811_baseline", CFS_READ))>0){
      cfs_seek(fd, 0, CFS_SEEK_SET);
      cfs_read(fd, buf, 10);
      cfs_close(fd);
      baseline = strtol(buf, NULL, 10);
      ccs811_sensor.configure(CCS811_CURRENT_BASELINE, baseline);
      PRINTF("Writing baseline with: %d", baseline);
    }
  }
#endif /* APP_CONFIG_STORAGE_COFFEE */

  return len;
}

static int
read_baseline(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
#if APP_CONFIG_STORAGE_COFFEE
  static int fd;
  char buf[10];
#endif
  uint16_t baseline=0;

  if( ccs811_sensor.status(CCS811_POWER_STATE) ){
    baseline = ccs811_sensor.status(CCS811_CURRENT_BASELINE);
    PRINTF("Reading baseline from chip: %d\n", baseline);
  }
#if APP_CONFIG_STORAGE_COFFEE
  else {
    if((fd = cfs_open("ccs811_baseline", CFS_READ))>0){
      cfs_seek(fd, 0, CFS_SEEK_SET);
      cfs_read(fd, buf, 10);
      cfs_close(fd);
      baseline = strtol(buf, NULL, 10);
      PRINTF("Reading baseline from memory: %d\n", baseline);
    }
  }
#endif
  return ctx->writer->write_int(ctx, outbuf, outsize, baseline);
}

static int
write_baseline(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
#if APP_CONFIG_STORAGE_COFFEE
    static int fd;
    char buf[10];
#endif /* APP_CONFIG_STORAGE_COFFEE */
  int32_t value;
  size_t len = 0;
  len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  ccs811_sensor.configure(CCS811_CURRENT_BASELINE, (uint16_t)value);
#if APP_CONFIG_STORAGE_COFFEE
      sprintf(buf, "%d", (uint16_t)value);
      fd = cfs_open("ccs811_baseline", CFS_WRITE);
      cfs_write(fd, buf, strlen(buf));
      cfs_close(fd);
      PRINTF("Writing baseline in memory: %s", buf);
#endif /* APP_CONFIG_STORAGE_COFFEE */
  return len;
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(co2_resource,
                /* concentration (Current) */
                LWM2M_RESOURCE_CALLBACK(5700, { sensor_co2_value, NULL, NULL }),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "ppm"),
                /* Min Range Value */
                LWM2M_RESOURCE_INTEGER(5603, IPSO_CO2_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_INTEGER(5604, IPSO_CO2_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5601, &min_co2_value),
                /* Max Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5602, &max_co2_value),

                LWM2M_RESOURCE_CALLBACK(REURES_SAMPLING_INTERVAL, { read_sampling, write_sampling, exec_sampling }),
                LWM2M_RESOURCE_CALLBACK(5850, {read_power_state, write_power_state, NULL}),
                LWM2M_RESOURCE_CALLBACK(8000, {read_baseline, write_baseline, NULL}),
                LWM2M_RESOURCE_STRING(5750, "CO2"),
                LWM2M_RESOURCE_STRING(5751, "MOX Gas Sensor")
                );

LWM2M_RESOURCES(tvoc_resource,
                /* concentration (Current) */
                LWM2M_RESOURCE_CALLBACK(5700, { sensor_tvoc_value, NULL, NULL }),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "ppb"),
                /* Min Range Value */
                LWM2M_RESOURCE_INTEGER(5603, IPSO_TVOC_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_INTEGER(5604, IPSO_TVOC_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5601, &min_tvoc_value),
                /* Max Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5602, &max_tvoc_value),

                LWM2M_RESOURCE_CALLBACK(REURES_SAMPLING_INTERVAL, { read_sampling, write_sampling, exec_sampling }),
                LWM2M_RESOURCE_CALLBACK(5850, {read_power_state, write_power_state, NULL}),
                LWM2M_RESOURCE_STRING(5750, "TVOC"),
                LWM2M_RESOURCE_STRING(5751, "MOX Gas Sensor")
                );
LWM2M_INSTANCES(concentration_instances,
                LWM2M_INSTANCE(0, co2_resource),
                LWM2M_INSTANCE(1, tvoc_resource)
                );
LWM2M_OBJECT(concentration, 3325, concentration_instances);
/*---------------------------------------------------------------------------*/
static int
read_sensor_value(int32_t *value, uint8_t type)
{
  uint16_t sensors_value[2];
  SENSORS_MEASURE(ccs811_sensor);
  sensors_value[0] = ccs811_sensor.value(CCS811_SENSE_CO2);
  sensors_value[1] = ccs811_sensor.value(CCS811_SENSE_TVOC);

  if(type == CO2){
    *value = sensors_value[0];
    if(*value < min_co2_value) {
      min_co2_value = *value;
    }
    if(*value > max_co2_value) {
      max_co2_value = *value;
    }
  } else if (type == TVOC){
    *value = sensors_value[1];
    if(*value < min_tvoc_value) {
      min_tvoc_value = *value;
    }
    if(*value > max_tvoc_value) {
      min_tvoc_value = *value;
    }
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static void
handle_periodic_timer(void *ptr)
{
  //static uint16_t last_co2_value = IPSO_CO2_MIN;
  //static uint16_t last_tvoc_value = IPSO_TVOC_MIN;

  uint16_t sensors_value[2];
  struct ctimer *periodic_timer = ptr;

  SENSORS_MEASURE(ccs811_sensor);
  sensors_value[0] = ccs811_sensor.value(CCS811_SENSE_CO2);
  sensors_value[1] = ccs811_sensor.value(CCS811_SENSE_TVOC);

  if(sensors_value[0] < min_co2_value) {
    min_co2_value = sensors_value[0];
    lwm2m_object_notify_observers(&concentration, "/0/5601");
  }
  if(sensors_value[0] > max_co2_value) {
    max_co2_value = sensors_value[0];
    lwm2m_object_notify_observers(&concentration, "/0/5602");
  }

  if(sensors_value[1] < min_tvoc_value) {
    min_tvoc_value = sensors_value[1];
    lwm2m_object_notify_observers(&concentration, "/1/5601");
  }

  if(sensors_value[1] > max_tvoc_value) {
    max_tvoc_value = sensors_value[1];
    lwm2m_object_notify_observers(&concentration, "/1/5602");
  }

  /* Only notify when the value has changed since last */
  //if(sensors_value[0] != last_co2_value) {
    //last_co2_value = sensors_value[0];
    lwm2m_object_notify_observers(&concentration, "/0/5700");
  //}

  //if(sensors_value[1] != last_tvoc_value) {
    //last_tvoc_value = sensors_value[1];
    lwm2m_object_notify_observers(&concentration, "/1/5700");
  //}

  ctimer_reset(periodic_timer);
}
/*---------------------------------------------------------------------------*/
void
ipso_concentration_init(void)
{
  min_co2_value = IPSO_CO2_MAX;
  max_co2_value = IPSO_CO2_MIN;
  min_tvoc_value = IPSO_TVOC_MAX;
  max_tvoc_value = IPSO_TVOC_MIN;

  add_sampling(&concentration, -1, handle_periodic_timer);

  /* register this device and its handlers - the handlers automatically
     sends in the object to handle */
  lwm2m_engine_register_object(&concentration);

}
/*---------------------------------------------------------------------------*/
/** @} */
