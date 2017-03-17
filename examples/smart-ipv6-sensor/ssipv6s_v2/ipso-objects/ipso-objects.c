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
 * \addtogroup oma-lwm2m
 * @{
 */

/**
 * \file
 *         Implementation of the IPSO Objects
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#include "contiki.h"
#include "ipso-objects.h"
/*---------------------------------------------------------------------------*/

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

struct sampler {
  uint8_t isused;
  uint8_t object_id;
  int8_t instance_id;
  int32_t interval;
  struct ctimer periodic_timer;
  void (* callback)(void *param);
};

#define LWM2M_MAX_SAMPLER       9

static struct sampler sampling_timers[LWM2M_MAX_SAMPLER];

void
add_sampling(const lwm2m_object_t *object, int8_t instance_id, void *callback)
{
  uint8_t i;
  struct sampler *smpl;

  for(i=0; i<LWM2M_MAX_SAMPLER; i++){
    if(sampling_timers[i].isused == 0){
      smpl = &sampling_timers[i];
      smpl->isused = 1;
      smpl->object_id = object->id;
      smpl->instance_id = instance_id;
      smpl->interval = 10;
      smpl->callback = callback;
      PRINTF("Add sampler for /%d/%d.\n", object->id, instance_id);
      break;
    }
  }
}

struct sampler*
lookup_sampler(uint8_t object_id, int8_t instance_id)
{
  uint8_t i;
  for(i=0; i<LWM2M_MAX_SAMPLER; i++){
    if(sampling_timers[i].isused)
    {
      if(sampling_timers[i].instance_id<0){
        if(sampling_timers[i].object_id == object_id)
        {
          return &sampling_timers[i];
        }
      } else {
        if(sampling_timers[i].object_id == object_id &&
           sampling_timers[i].instance_id == instance_id)
        {
          return &sampling_timers[i];
        }
      }
    }
  }
  return NULL;
}


int
read_sampling(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  struct sampler *smpl;
  smpl = lookup_sampler(ctx->object_id, ctx->object_instance_id);
  if(smpl==NULL){
    return 0;
  }

  return ctx->writer->write_int(ctx, outbuf, outsize, smpl->interval);
  return 1;
}

int
write_sampling(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  size_t len = 0;
  struct sampler *smpl;

  smpl = lookup_sampler(ctx->object_id, ctx->object_instance_id);
  if(smpl==NULL){
    return 0;
  }

  len = ctx->reader->read_int(ctx, inbuf, insize, &value);

  /*if(ctx)
    len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  else
    value = 0;
*/
  // setting value to 0 stop the timer but doesn't change the parameter
  if(value){
    smpl->interval = value;
  }

  // change the sampling only if the timer is already started
  if(value && smpl->periodic_timer.etimer.p != PROCESS_NONE){
    ctimer_set(&smpl->periodic_timer, CLOCK_SECOND * smpl->interval, smpl->callback, &smpl->periodic_timer);
  } else {
    ctimer_stop(&smpl->periodic_timer);
  }

  return len;
  return 1;
}

// used to start the sampling automatically when receiving GET with observation
int
exec_sampling(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  struct sampler *smpl;

  smpl = lookup_sampler(ctx->object_id, ctx->object_instance_id);
  if(smpl==NULL){
    return 0;
  }

  ctimer_set(&smpl->periodic_timer, CLOCK_SECOND * smpl->interval, smpl->callback, &smpl->periodic_timer);
  return 1;
}


void
ipso_objects_init(void)
{
  /* initialize any relevant object for the IPSO Objects */
  ipso_temperature_init();
  ipso_humidity_init();
  ipso_illuminance_init();
  ipso_barometer_init();
  ipso_voltage_init();
  ipso_current_init();
  ipso_presence_init();
  ipso_microclap_init();
  ipso_concentration_init();
  ipso_energest_init();
  lwm2m_connectivity_init();
}
/*---------------------------------------------------------------------------*/
/** @} */
