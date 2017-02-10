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
 *         Implementation of proprietary Microphone object
 * \author
 *         Darko Petrovic <darko.petrovic@hevs.ch>
 *
 */

#include <stdint.h>
#include "ipso-objects.h"
#include "lwm2m-object.h"
#include "lwm2m-engine.h"
#include "er-coap-engine.h"

#include "mic-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

static uint32_t detection_counter;
static struct ctimer microclap_timer;

static int
read_power_state(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int real_power_status = mic_sensor.status(MIC_POWER);
  return ctx->writer->write_int(ctx, outbuf, outsize, real_power_status);
}

static int
write_power_state(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  size_t len;
  len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  mic_sensor.configure(SENSORS_ACTIVE, value);
  return len;
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(microclap_resources,
                LWM2M_RESOURCE_INTEGER_VAR(5501, &detection_counter),
                LWM2M_RESOURCE_STRING(5751, "Microphone"),
                LWM2M_RESOURCE_CALLBACK(5850, {read_power_state, write_power_state, NULL})
                );
LWM2M_INSTANCES(microclap_instances,
                LWM2M_INSTANCE(0, microclap_resources));
LWM2M_OBJECT(microclap, OBJ_MICROCLAP, microclap_instances);
/*---------------------------------------------------------------------------*/

static void
microclap_timeout(void* ptr)
{
  lwm2m_object_notify_observers(&microclap, "/0/5501");
  detection_counter = 0;
}

void
ipso_microclap_detection(void)
{
  detection_counter++;
  ctimer_set(&microclap_timer, CLOCK_SECOND, microclap_timeout, NULL);
}

/*---------------------------------------------------------------------------*/
void
ipso_microclap_init(void)
{
  /* register this device and its handlers - the handlers automatically
     sends in the object to handle */
  lwm2m_engine_register_object(&microclap);
}
/*---------------------------------------------------------------------------*/
/** @} */
