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
 *         Darko Petrovic
 */

#include "contiki.h"
#include "ipso-objects.h"
/*---------------------------------------------------------------------------*/

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define ENABLE_RESOURCE_DESYNCHRONISATION   1

struct sampler {
  uint8_t isused;
  uint16_t object_id;
  int16_t instance_id;
  int32_t interval;
  struct ctimer periodic_timer;
  void (* callback)(void *param);
};

#define LWM2M_MAX_SAMPLER       9

static struct sampler sampling_timers[LWM2M_MAX_SAMPLER];
static struct ctimer desynch_delay;

#if DEBUG && ENABLE_RESOURCE_DESYNCHRONISATION
static char *
float2str(float num, uint8_t preci)
{
  int integer=(int)num, decimal=0;
  static char buf[20];
  preci = preci > 10 ? 10 : preci;
  num -= integer;
  while((num != 0) && (preci-- > 0)) {
    decimal *= 10;
    num *= 10;
    decimal += (int)num;
    num -= (int)num;
  }
  switch(preci){
    case 1:
      sprintf(buf, "%d.%01d", integer, decimal);
      break;
    case 2:
      sprintf(buf, "%d.%02d", integer, decimal);
      break;
    case 3:
      sprintf(buf, "%d.%03d", integer, decimal);
      break;
    default:
      sprintf(buf, "%d.%01d", integer, decimal);
  }
  return buf;
}
#endif

#if ENABLE_RESOURCE_DESYNCHRONISATION
static int
gcd( int a, int b )
{
  int c;
  while ( a != 0 ) {
     c = a; a = b%a;  b = c;
  }
  return b;
}

static int
gcd_a(int n, int a[n])
{
    if(n==1) return a[0];
    if(n==2) return gcd(a[0], a[1]);
    int h = n / 2;
    return gcd(gcd_a(h, &a[h-1]), gcd_a(n - h, &a[h]));
}

static void
desynch_samplers(void *ptr)
{
  struct sampler *smpl;
  clock_time_t tdiff;
  uint32_t intervals[LWM2M_MAX_SAMPLER];
  struct sampler *samplers[LWM2M_MAX_SAMPLER];
  struct sampler *samplers_p = NULL;
  uint32_t best_interval = 0;
  uint8_t active_periodic_resources;
  uint8_t i, j;

  PRINTF("Desynchronizing samplers...\n");

  active_periodic_resources = 0;

  for(i=0; i<LWM2M_MAX_SAMPLER; i++){
    if(sampling_timers[i].isused == 1){
      smpl = &sampling_timers[i];
      if(smpl->periodic_timer.etimer.p != PROCESS_NONE){
        intervals[active_periodic_resources] = smpl->interval*CLOCK_SECOND;
        PRINTF("Resource interval %lu\n", smpl->interval);
        samplers[active_periodic_resources++] = smpl;
      }
    }
  }

  if( active_periodic_resources > 1 ){

    /* Calculate the best interval between resources. */
    best_interval = gcd_a(active_periodic_resources, (int*)intervals)/active_periodic_resources;

    /* Ascending order of the periodic resources by the remaining time. */
    for (i = 0; i < active_periodic_resources; ++i)
    {
      for (j = i + 1; j < active_periodic_resources; ++j)
      {
        if (timer_remaining(&(samplers[i]->periodic_timer.etimer.timer)) >
          timer_remaining(&(samplers[j]->periodic_timer.etimer.timer)))
        {
          samplers_p =  samplers[i];
          samplers[i] = samplers[j];
          samplers[j] = samplers_p;
        }
      }
    }

    /* Adjust resources periodic wake-up */
    for (i = 0; i < active_periodic_resources-1; ++i)
    {
      tdiff = timer_remaining(&(samplers[i+1]->periodic_timer.etimer.timer)) - \
          timer_remaining(&(samplers[i]->periodic_timer.etimer.timer));

      if((int32_t)tdiff < 0){
        tdiff = 0;
      }
      etimer_adjust(&samplers[i+1]->periodic_timer.etimer, best_interval-tdiff);
    } /* for() */

  } /* if() */

#if DEBUG
  PRINTF("RES: Best interval between resources (%d) is %lu (%s seconds).\n", active_periodic_resources,
      best_interval, float2str((float)best_interval/CLOCK_SECOND, 1));
  for (i = 0; i < active_periodic_resources; ++i) {
    PRINTF("RES: Resource '/%d/%d' wakes-up in %s seconds.\n", samplers[i]->object_id,  samplers[i]->instance_id,
        float2str((float)timer_remaining(&(samplers[i]->periodic_timer.etimer.timer))/CLOCK_SECOND, 1));
  }
#endif
}
#endif

void
add_sampling(const lwm2m_object_t *object, int16_t instance_id, void *callback)
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
      PRINTF("Add sampler for /%d/%d.\n", smpl->object_id, smpl->instance_id);
      break;
    }
  }
}

struct sampler*
lookup_sampler(uint16_t object_id, int16_t instance_id)
{
  uint8_t i;
  PRINTF("Looking for sampler /%d/%d.\n", object_id, instance_id);
  for(i=0; i<LWM2M_MAX_SAMPLER; i++){
    if(sampling_timers[i].isused)
    {
      if(sampling_timers[i].instance_id<0){
        if(sampling_timers[i].object_id == object_id)
        {
          PRINTF("Sampler found (shared between instances)\n");
          return &sampling_timers[i];
        }
      } else {
        if(sampling_timers[i].object_id == object_id &&
           sampling_timers[i].instance_id == instance_id)
        {
          PRINTF("Sampler found\n");
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

  // The observer_periodic use this function internally to stop the periodic timer
  // and set 'inbuf' to NULL in that case.
  if(inbuf != NULL){
    len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  } else {
    value = 0;
  }

  // setting value to 0 stop the timer but doesn't change the parameter
  if(value){
    smpl->interval = value;
  }

  // change the sampling only if the timer is already started
  if(value && smpl->periodic_timer.etimer.p != PROCESS_NONE){
    ctimer_set(&smpl->periodic_timer, CLOCK_SECOND * smpl->interval, smpl->callback, &smpl->periodic_timer);
#if ENABLE_RESOURCE_DESYNCHRONISATION
  ctimer_set(&desynch_delay, CLOCK_SECOND * 10, desynch_samplers, NULL);
#endif /* ENABLE_RESOURCE_DESYNCHRONISATION */
  } else {
    ctimer_stop(&smpl->periodic_timer);
  }

  return len;
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

#if ENABLE_RESOURCE_DESYNCHRONISATION
  ctimer_set(&desynch_delay, CLOCK_SECOND *10, desynch_samplers, NULL);
#endif /* ENABLE_RESOURCE_DESYNCHRONISATION */

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
