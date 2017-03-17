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
 *         Implementation of OMA LWM2M / IPSO Barometer
 * \author
 *         Darko Petrovic <darko.petrovic@hevs.ch>
 *
 */

#include <stdint.h>
#include "ipso-objects.h"
#include "bmp280-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define IPSO_PRESSURE_MIN 30000
#define IPSO_PRESSURE_MAX 110000

static int32_t min_sensor_value;
static int32_t max_sensor_value;
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

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(barometer_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_CALLBACK(5700, { sensor_value, NULL, NULL }),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "Pa"),
                /* Min Range Value */
                LWM2M_RESOURCE_INTEGER(5603, IPSO_PRESSURE_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_INTEGER(5604, IPSO_PRESSURE_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5601, &min_sensor_value),
                /* Max Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5602, &max_sensor_value),

                LWM2M_RESOURCE_CALLBACK(REURES_SAMPLING_INTERVAL, { read_sampling, write_sampling, exec_sampling })
                );
LWM2M_INSTANCES(barometer_instances,
                LWM2M_INSTANCE(0, barometer_resources));
LWM2M_OBJECT(barometer, 3315, barometer_instances);
/*---------------------------------------------------------------------------*/
static int
read_sensor_value(int32_t *value)
{
  uint32_t sensors_value;

  // read sensor value here
  SENSORS_ACTIVATE(bmp280_sensor);
  SENSORS_MEASURE(bmp280_sensor);
  sensors_value = bmp280_sensor.value( BMP280_PRESSURE );
  SENSORS_DEACTIVATE(bmp280_sensor);

  /* Convert to fix float */
  *value = sensors_value;

  if(*value < min_sensor_value) {
    min_sensor_value = *value;
    lwm2m_object_notify_observers(&barometer, "/0/5601");
  }
  if(*value > max_sensor_value) {
    max_sensor_value = *value;
    lwm2m_object_notify_observers(&barometer, "/0/5602");
  }
  return 1;

}
/*---------------------------------------------------------------------------*/
static void
handle_periodic_timer(void *ptr)
{
  struct ctimer *periodic_timer = ptr;
  lwm2m_object_notify_observers(&barometer, "/0/5700");
  ctimer_reset(periodic_timer);
}
/*---------------------------------------------------------------------------*/
void
ipso_barometer_init(void)
{
  min_sensor_value = IPSO_PRESSURE_MAX;
  max_sensor_value = IPSO_PRESSURE_MIN;

  add_sampling(&barometer, 0, handle_periodic_timer);

  /* register this device and its handlers - the handlers automatically
     sends in the object to handle */
  lwm2m_engine_register_object(&barometer);
}
/*---------------------------------------------------------------------------*/
/** @} */
