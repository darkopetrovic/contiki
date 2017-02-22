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
 *
 */

/**
 * \file
 *         Implementation of the Contiki OMA LWM2M server
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#include <stdint.h>
#include "lwm2m-object.h"
#include "lwm2m-engine.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static int32_t sid;

static int
read_lifetime(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, lwm2m_client.lifetime);
}

static int
write_lifetime(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  uint32_t value;
  size_t len;

  len = ctx->reader->read_int(ctx, inbuf, insize, (int32_t *)&value);
  lwm2m_engine_update_registration(value, NULL);

  return len;
}

static int
read_binding(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_string(ctx, outbuf, outsize, lwm2m_client.binding, strlen(lwm2m_client.binding));
}

static int
write_binding(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
            uint8_t *outbuf, size_t outsize)
{
  char value[5];
  size_t len;

  len = ctx->reader->read_string(ctx, inbuf, insize, (uint8_t *)value, sizeof(value));
  lwm2m_engine_update_registration(0, (const char*)value);

#if RDC_SLEEPING_HOST
  if(strchr((const char*)value, 'Q')){
    if(!USB_IS_PLUGGED()){
      crdc_disable_rdc_delay(0, 3);
    }
  } else {
    crdc_clear_stop_rdc_timer();
    crdc_enable_rdc();
  }
#endif

  return len;
}

LWM2M_RESOURCES(server_resources,
		LWM2M_RESOURCE_INTEGER_VAR(0, &sid),
		LWM2M_RESOURCE_CALLBACK(1, { read_lifetime, write_lifetime, NULL }),
		LWM2M_RESOURCE_CALLBACK(7, { read_binding, write_binding, NULL }),

                );
LWM2M_INSTANCES(server_instances, LWM2M_INSTANCE(0, server_resources));
LWM2M_OBJECT(server, 1, server_instances);
/*---------------------------------------------------------------------------*/
void
lwm2m_server_init(void)
{
  /**
   * Register this device and its handlers - the handlers
   * automatically sends in the object to handle
   */
  PRINTF("*** Init lwm2m-server\n");
  lwm2m_engine_register_object(&server);
}
/*---------------------------------------------------------------------------*/
/** @} */
