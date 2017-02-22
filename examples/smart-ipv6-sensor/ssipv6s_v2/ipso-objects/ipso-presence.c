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
 *         Implementation of OMA LWM2M / IPSO Presence
 * \author
 *         Darko Petrovic <darko.petrovic@hevs.ch>
 *
 */

#include <stdint.h>
#include "ipso-objects.h"
#include "lwm2m-object.h"
#include "lwm2m-engine.h"
#include "er-coap-engine.h"

#include "pir-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define DEFAULT_SLEEP_DURATION   30000 // ms

static uint8_t input_state;
static uint32_t detection_counter;
static uint32_t sleep_duration_ms = DEFAULT_SLEEP_DURATION;
static struct ctimer pir_sleep;

static int
read_state(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_boolean(ctx, outbuf, outsize, input_state);
}

static int
read_power_state(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int real_power_status = pir_sensor.status(PIR_POWER);
  return ctx->writer->write_int(ctx, outbuf, outsize, real_power_status);
}

static int
write_power_state(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  size_t len;
  len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  pir_sensor.configure(SENSORS_ACTIVE, value);
  return len;
}

static int
read_sleep_duration(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, sleep_duration_ms);
}

static int
write_sleep_duration(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  size_t len;
  len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  sleep_duration_ms = value;
  pir_sensor.configure(PIR_DEACTIVATION_DELAY, sleep_duration_ms/1000);
  return len;
}

static int
reset_counter(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  detection_counter = 0;
  return 1;
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(presence_resources,
                LWM2M_RESOURCE_CALLBACK(5500, { read_state, NULL, NULL }),
                LWM2M_RESOURCE_INTEGER_VAR(5501, &detection_counter),
                LWM2M_RESOURCE_CALLBACK(5505, {NULL, NULL, reset_counter}),
                LWM2M_RESOURCE_STRING(5751, "PIR"),
                LWM2M_RESOURCE_CALLBACK(5903, {read_sleep_duration, write_sleep_duration, NULL}),
                LWM2M_RESOURCE_CALLBACK(5850, {read_power_state, write_power_state, NULL})
                );
LWM2M_INSTANCES(presence_instances,
                LWM2M_INSTANCE(0, presence_resources));
LWM2M_OBJECT(presence, 3302, presence_instances);
/*---------------------------------------------------------------------------*/

static void
reset_state(void *ptr)
{
  input_state = 0;
}

void
ipso_presence_detection(void)
{
  detection_counter++;
  input_state = 1;
  lwm2m_object_notify_observers(&presence, "/0/5500");
  ctimer_set(&pir_sleep, CLOCK_SECOND * pir_sensor.status(PIR_DEACTIVATION_DELAY), reset_state, NULL);
}

/*---------------------------------------------------------------------------*/
void
ipso_presence_init(void)
{

  pir_sensor.configure(PIR_DEACTIVATION_DELAY, sleep_duration_ms/1000);
  /* register this device and its handlers - the handlers automatically
     sends in the object to handle */
  lwm2m_engine_register_object(&presence);
}
/*---------------------------------------------------------------------------*/
/** @} */
