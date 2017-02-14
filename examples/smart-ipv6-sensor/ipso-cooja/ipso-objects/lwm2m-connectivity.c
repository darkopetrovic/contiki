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
 *         Implementation of the Contiki OMA LWM2M Connectivity Monitoring
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#include "lwm2m-object.h"
#include "lwm2m-engine.h"

#include "net/ip/ip64-addr.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static char *
ipaddr_print(const uip_ipaddr_t *addr)
{
  static char ipaddr_buf[40], *ipaddr_bufptr;

#if NETSTACK_CONF_WITH_IPV6
  uint16_t a;
  unsigned int i;
  int f;
#endif /* NETSTACK_CONF_WITH_IPV6 */
  if(addr == NULL) {
    return "(NULL IP addr)";
  }
#if NETSTACK_CONF_WITH_IPV6
  if(ip64_addr_is_ipv4_mapped_addr(addr)) {
    /*
     * Printing IPv4-mapped addresses is done according to RFC 4291 [1]
     *
     *     "An alternative form that is sometimes more
     *     convenient when dealing with a mixed environment
     *     of IPv4 and IPv6 nodes is x:x:x:x:x:x:d.d.d.d,
     *     where the 'x's are the hexadecimal values of the
     *     six high-order 16-bit pieces of the address, and
     *     the 'd's are the decimal values of the four
     *     low-order 8-bit pieces of the address (standard
     *     IPv4 representation)."
     *
     * [1] https://tools.ietf.org/html/rfc4291#page-4
     */
    sprintf(ipaddr_buf, "::FFFF:%u.%u.%u.%u", addr->u8[12], addr->u8[13], addr->u8[14], addr->u8[15]);
  } else {
    ipaddr_bufptr = ipaddr_buf;
    for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
      a = (addr->u8[i] << 8) + addr->u8[i + 1];
      if(a == 0 && f >= 0) {
        if(f++ == 0) {
          ipaddr_bufptr += sprintf(ipaddr_bufptr, "::");
        }
      } else {
        if(f > 0) {
          f = -1;
        } else if(i > 0) {
          ipaddr_bufptr += sprintf(ipaddr_bufptr, ":");
        }
        ipaddr_bufptr += sprintf(ipaddr_bufptr, "%x", a);
      }
    }
  }
#else /* NETSTACK_CONF_WITH_IPV6 */
  sprintf(ipaddr_buf, "%u.%u.%u.%u", addr->u8[0], addr->u8[1], addr->u8[2], addr->u8[3]);
#endif /* NETSTACK_CONF_WITH_IPV6 */

  return ipaddr_buf;
}

static int
read_rssi(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  radio_value_t rssi;
  NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &rssi);
  return ctx->writer->write_int(ctx, outbuf, outsize, rssi);
}

static int
read_lqi(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  radio_value_t lqi;
  NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_LINK_QUALITY, &lqi);
  return ctx->writer->write_int(ctx, outbuf, outsize, lqi);
}

static int
read_ipaddr(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  char buf[60];
  uip_ds6_addr_t * global = uip_ds6_get_global(ADDR_PREFERRED);
  sprintf(buf, "%s", ipaddr_print(&global->ipaddr));
  return ctx->writer->write_string(ctx, outbuf, outsize, buf, strlen(buf));
}

static int
read_nexthop(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  char buf[60];
  uip_ipaddr_t *ipaddr = uip_ds6_defrt_choose();
  sprintf(buf, "%s", ipaddr_print(ipaddr));
  return ctx->writer->write_string(ctx, outbuf, outsize, buf, strlen(buf));
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(connectivity_resources,
                LWM2M_RESOURCE_INTEGER(0, 23),
                LWM2M_RESOURCE_INTEGER(1, 23),
                LWM2M_RESOURCE_CALLBACK(2, {read_rssi, NULL, NULL}),
                LWM2M_RESOURCE_CALLBACK(3, {read_lqi, NULL, NULL}),
                LWM2M_RESOURCE_CALLBACK(4, {read_ipaddr, NULL, NULL}),
                LWM2M_RESOURCE_CALLBACK(5, {read_nexthop, NULL, NULL})
                );
LWM2M_INSTANCES(connectivity_instances, LWM2M_INSTANCE(0, connectivity_resources));
LWM2M_OBJECT(connectivity, 4, connectivity_instances);
/*---------------------------------------------------------------------------*/
void
lwm2m_connectivity_init(void)
{
  PRINTF("*** Init lwm2m-connectivity\n");
  lwm2m_engine_register_object(&connectivity);
}
/*---------------------------------------------------------------------------*/
/** @} */
