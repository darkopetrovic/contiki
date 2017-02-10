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
 *         Implementation of OMA LWM2M / IPSO Illuminance
 * \author
 *         Darko Petrovic <darko.petrovic@hevs.ch>
 *
 */

#include <stdint.h>
#include "ipso-objects.h"
#include "lwm2m-object.h"
#include "lwm2m-engine.h"
#include "er-coap-engine.h"

#include "tsl2561-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define IPSO_ILLUMINANCE_MIN    0
#define IPSO_ILLUMINANCE_MAX    40000

static struct ctimer periodic_timer;
static int32_t min_sensor_value;
static int32_t max_sensor_value;
static int32_t interval=10;
static int read_sensor_value(int32_t *value);
static void handle_periodic_timer(void *ptr);
/*---------------------------------------------------------------------------*/
static int
sensor_value(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  if(read_sensor_value(&value)) {
    return ctx->writer->write_int(ctx, outbuf, outsize, value);
  }
  return 0;
}

static int
read_sampling(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, interval);
}

static int
write_sampling(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  size_t len = 0;

  if(ctx)
    len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  else
    value = 0;

  // setting value to 0 stop the timer but doesn't change the parameter
  if(value){
    interval = value;
  }

  if(value && periodic_timer.etimer.p != PROCESS_NONE){
    ctimer_set(&periodic_timer, CLOCK_SECOND * interval, handle_periodic_timer, NULL);
  } else {
    ctimer_stop(&periodic_timer);
  }

  return len;
}

static int
exec_sampling(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  ctimer_set(&periodic_timer, CLOCK_SECOND * interval, handle_periodic_timer, NULL);
  return 1;
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(illuminance_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_CALLBACK(5700, { sensor_value, NULL, NULL }),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "Lux"),
                /* Min Range Value */
                LWM2M_RESOURCE_INTEGER(5603, IPSO_ILLUMINANCE_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_INTEGER(5604, IPSO_ILLUMINANCE_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5601, &min_sensor_value),
                /* Max Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5602, &max_sensor_value),

                LWM2M_RESOURCE_CALLBACK(IPSO_RES_SAMPLING_INTERVAL, { read_sampling, write_sampling, exec_sampling })
                );
LWM2M_INSTANCES(illuminance_instances,
                LWM2M_INSTANCE(0, illuminance_resources));
LWM2M_OBJECT(illuminance, 3301, illuminance_instances);
/*---------------------------------------------------------------------------*/
static int
read_sensor_value(int32_t *value)
{
  uint16_t sensors_value;

  // read sensor value here
  SENSORS_ACTIVATE(tsl2561_sensor);
  SENSORS_MEASURE(tsl2561_sensor);
  sensors_value = tsl2561_sensor.value( TSL2561_LUX );
  SENSORS_DEACTIVATE(tsl2561_sensor);

  /* Convert to fix float */
  *value = sensors_value;

  if(*value < min_sensor_value) {
    min_sensor_value = *value;
    lwm2m_object_notify_observers(&illuminance, "/0/5601");
  }
  if(*value > max_sensor_value) {
    max_sensor_value = *value;
    lwm2m_object_notify_observers(&illuminance, "/0/5602");
  }
  return 1;

}
/*---------------------------------------------------------------------------*/
static void
handle_periodic_timer(void *ptr)
{
  static int32_t last_value = IPSO_ILLUMINANCE_MIN;
  int32_t v;

  /* Only notify when the value has changed since last */
  if(read_sensor_value(&v) && v != last_value) {
    last_value = v;
    lwm2m_object_notify_observers(&illuminance, "/0/5700");
  }
  ctimer_reset(&periodic_timer);
}
/*---------------------------------------------------------------------------*/
void
ipso_illuminance_init(void)
{
  min_sensor_value = IPSO_ILLUMINANCE_MAX;
  max_sensor_value = IPSO_ILLUMINANCE_MIN;

  /* register this device and its handlers - the handlers automatically
     sends in the object to handle */
  lwm2m_engine_register_object(&illuminance);
}
/*---------------------------------------------------------------------------*/
/** @} */
