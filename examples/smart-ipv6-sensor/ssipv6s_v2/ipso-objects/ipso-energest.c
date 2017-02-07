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
 *
 * \author
 *
 */

#include "lwm2m-object.h"
#include "lwm2m-engine.h"
#include "ipso-objects.h"

#include "power-track.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static int32_t sampling_interval=10;
static uint8_t power_state;

static int
read_samping(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, sampling_interval);
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
    sampling_interval = value;
  }

  return len;
}

static int
exec_sampling(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  powertrack_start(sampling_interval*CLOCK_SECOND);
  power_state = 1;
  return 1;
}

static int
reset_counter(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  powertrack_reset();
  return 1;
}

static int
read_power_state(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, power_state);
}

static int
write_power_state(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  size_t len;
  len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  power_state = value;

  if(power_state){
    powertrack_start(sampling_interval*CLOCK_SECOND);
  } else {
    powertrack_stop();
  }

  return len;
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(energest_resources,
          LWM2M_RESOURCE_FLOATFIX_VAR(8001, &energest_data.charge_consumed),
          LWM2M_RESOURCE_INTEGER_VAR(8010, &energest_data_repr.all_cpu),
          LWM2M_RESOURCE_INTEGER_VAR(8011, &energest_data_repr.all_lpm),
          LWM2M_RESOURCE_INTEGER_VAR(8021, &energest_data_repr.all_transmit),
          LWM2M_RESOURCE_INTEGER_VAR(8022, &energest_data_repr.all_listen),
          LWM2M_RESOURCE_INTEGER_VAR(8030, &energest_data_repr.all_leds),
          LWM2M_RESOURCE_INTEGER_VAR(8040, &energest_data_repr.all_sensors_ina3221),
          LWM2M_RESOURCE_INTEGER_VAR(8041, &energest_data_repr.all_sensors_sht21),
          /*LWM2M_RESOURCE_INTEGER_VAR(8042, &energest_data_repr.all_sensors_pir),
          LWM2M_RESOURCE_INTEGER_VAR(8043, &energest_data_repr.all_sensors_bmp280),
          LWM2M_RESOURCE_INTEGER_VAR(8044, &energest_data_repr.all_sensors_tsl2561),
          LWM2M_RESOURCE_INTEGER_VAR(8045, &energest_data_repr.all_sensors_ccs811),
          LWM2M_RESOURCE_INTEGER_VAR(8046, &energest_data_repr.all_sensors_mic),*/
          LWM2M_RESOURCE_CALLBACK(5850, {read_power_state, write_power_state, NULL}),
          LWM2M_RESOURCE_CALLBACK(IPSO_RES_SAMPLING_INTERVAL, { read_samping, write_sampling, exec_sampling}),
          LWM2M_RESOURCE_CALLBACK(8100, { NULL, NULL, reset_counter})
                );
LWM2M_INSTANCES(energest_instances, LWM2M_INSTANCE(0, energest_resources));
LWM2M_OBJECT(energest, 7500, energest_instances);
/*---------------------------------------------------------------------------*/
void
ipso_energest_init(void)
{
  /**
   * Register this device and its handlers - the handlers
   * automatically sends in the object to handle.
   */
  lwm2m_engine_register_object(&energest);
}
/*---------------------------------------------------------------------------*/
/** @} */
