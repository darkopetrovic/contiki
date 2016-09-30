/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Contiki shell command ifconfig
 * \author
 *         Darko Petrovic
 */

#include "contiki.h"
#include "shell-ifconfig.h"

#include "net/ipv6/uip-ds6.h"
#include "net/ip/ip64-addr.h"

#if UIP_CONF_IPV6_RPL
#include "net/rpl/rpl.h"
#endif /* UIP_CONF_IPV6_RPL */

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(shell_ifconfig_process, "ifconfig");
SHELL_COMMAND(ifconfig_command, "ifconfig",
    "ifconfig : show interface configuration", &shell_ifconfig_process);
/*---------------------------------------------------------------------------*/

static char * ipaddr_print(const uip_ipaddr_t *addr);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(shell_ifconfig_process, ev, data)
{
  uint8_t i;
  uip_ds6_defrt_t *d;
  uip_ds6_element_t *element;
  uip_ds6_nbr_t *nbr;
  uip_ds6_prefix_t *prefix;
  char buf[120], *bufptr;

#if CONF_6LOWPAN_ND
  uip_ds6_border_router_t *br;
#endif

  PROCESS_BEGIN();

  /***** Interface configuration *****/

  sprintf(buf,
      "Interface configuration:\n"
      "    Link MTU: %lu\n"
      "    Hop limit: %d",
      uip_ds6_if.link_mtu,
      uip_ds6_if.cur_hop_limit);
  shell_output_str(&ifconfig_command, buf, "");

  sprintf(buf,
      "    Base reachable time: %lums\n"
      "    Reachable time: %lums",
      uip_ds6_if.base_reachable_time,
      uip_ds6_if.reachable_time);
  shell_output_str(&ifconfig_command, buf, "");

  sprintf(buf,
      "    Retransmission timer: %lums\n"
      "    Max DADNS: %d",
      uip_ds6_if.retrans_timer,
      uip_ds6_if.maxdadns);
  shell_output_str(&ifconfig_command, buf, "");

  /***** MAC Address *****/

  bufptr = buf;
  bufptr += sprintf(bufptr, "MAC Address:\n    ");
  for (i = 0; i < LINKADDR_SIZE - 1; i++) {
    bufptr += sprintf(bufptr, "%02x:", linkaddr_node_addr.u8[i]);
  }
  bufptr += sprintf(bufptr, "%02x", linkaddr_node_addr.u8[i]);
  shell_output_str(&ifconfig_command, buf, "");

  /***** Prefixes list *****/

  
  sprintf(buf, "Prefixes list (max %d):", UIP_DS6_PREFIX_NB);
  shell_output_str(&ifconfig_command, buf, "");

  for (prefix = uip_ds6_prefix_list;
      prefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB; prefix++) 
  {
    if (prefix->isused) {
      bufptr = buf;
      bufptr += sprintf(bufptr, "    %s/%d", ipaddr_print(&prefix->ipaddr), prefix->length);
      bufptr += sprintf(bufptr, " - lifetime: ");

      if( prefix->isinfinite ) {
        bufptr += sprintf(bufptr, "infinite");
      } else {
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER || CONF_6LOWPAN_ND
#if CONF_6LOWPAN_ND
        bufptr += sprintf(bufptr, "%lus (expiring in: %lus)",
            prefix->vlifetime.interval, stimer_remaining(&prefix->vlifetime));
#else /* CONF_6LOWPAN_ND */
        bufptr += sprintf(bufptr, "%lus", prefix->vlifetime);
#endif /* CONF_6LOWPAN_ND */
#else /* UIP_CONF_ROUTER */
        bufptr += sprintf(bufptr, "%lus (expiring in: %lus)",
            prefix->vlifetime.interval, stimer_remaining(&prefix->vlifetime));
#endif /* UIP_CONF_ROUTER */
      }
    shell_output_str(&ifconfig_command, buf, "");
    }
  }

  /***** Addresses list *****/

  sprintf(buf, "Addresses list (max: %d):", UIP_DS6_ADDR_NB);
  shell_output_str(&ifconfig_command, buf, "");


  for (element = (uip_ds6_element_t *) uip_ds6_if.addr_list;
      element
          < (uip_ds6_element_t *) ((uint8_t *) (uip_ds6_element_t *) uip_ds6_if.addr_list
              + (UIP_DS6_ADDR_NB * sizeof(uip_ds6_addr_t)));
      element = (uip_ds6_element_t *) ((uint8_t *) element
          + sizeof(uip_ds6_addr_t))) 
  {
    if (element->isused) {
      bufptr = buf;

      uip_ds6_addr_t *locaddr = (uip_ds6_addr_t *) element;
      bufptr += sprintf(bufptr, "    %s", ipaddr_print(&element->ipaddr));

      bufptr += sprintf(bufptr, " - state: ");
      switch (locaddr->state) {
      case ADDR_TENTATIVE:
        bufptr += sprintf(bufptr, "TENTATIVE");
        break;
      case ADDR_PREFERRED:
        bufptr += sprintf(bufptr, "PREFERRED");
        break;
      case ADDR_DEPRECATED:
        bufptr += sprintf(bufptr, "DEPRECATED");
        break;
      }

      bufptr += sprintf(bufptr, " - type: ");
      switch (locaddr->type) {
      case ADDR_AUTOCONF:
        bufptr += sprintf(bufptr, "Auto");
        break;
      case ADDR_DHCP:
        bufptr += sprintf(bufptr, "DHCP");
        break;
      case ADDR_MANUAL:
        bufptr += sprintf(bufptr, "Manual");
        break;
      }

      bufptr += sprintf(bufptr, " - lifetime: ");
      if (locaddr->isinfinite) {
        bufptr += sprintf(bufptr, "infinite");
      } else {
        bufptr += sprintf(bufptr, "%lus (expiring in: %lus)",
            locaddr->vlifetime.interval, stimer_remaining(&locaddr->vlifetime));
      }
    shell_output_str(&ifconfig_command, buf, "");
    }
  }

  /***** Default routers *****/

  sprintf(buf, "Default routers (max: %d):", UIP_DS6_DEFRT_NB);
  shell_output_str(&ifconfig_command, buf, "");

  for (d = uip_ds6_defrt_list_head(); d != NULL; d = list_item_next(d)) 
  {
    bufptr = buf;
    bufptr += sprintf(bufptr, "    %s - lifetime: ", ipaddr_print(&d->ipaddr));
    if (d->isinfinite) {
      bufptr += sprintf(bufptr, "infinite");
    } else {
      bufptr += sprintf(bufptr, "%lus (expiring in: %lus)",
          d->lifetime.interval, stimer_remaining(&d->lifetime));
    }
    shell_output_str(&ifconfig_command, buf, "");
  }

  /***** Neighbors list *****/

  sprintf(buf, "Neighbors list (max: %d):", NBR_TABLE_MAX_NEIGHBORS);
  shell_output_str(&ifconfig_command, buf, "");

  nbr = nbr_table_head(ds6_neighbors);
  while (nbr != NULL) 
  {
    bufptr = buf;
    bufptr += sprintf(bufptr, "    %s - deft. router: ", ipaddr_print(&nbr->ipaddr));

#if CONF_6LOWPAN_ND
    switch (nbr->isrouter) {
      case ISROUTER_RPL:
      case ISROUTER_YES:
        bufptr += sprintf(bufptr, "YES");
        break;
      case ISROUTER_NO:
        bufptr += sprintf(bufptr, "NO");
        break;
#if UIP_CONF_ROUTER
      case ISROUTER_NODEFINE:
        bufptr += sprintf(bufptr, "UNDEFINED YET");
        break;
#endif /* UIP_CONF_ROUTER */
    }
#else /* CONF_6LOWPAN_ND */
    if (nbr->isrouter) {
      bufptr += sprintf(bufptr, "YES");
    } else {
      bufptr += sprintf(bufptr, "NO");
    }
#endif /* CONF_6LOWPAN_ND */

    bufptr += sprintf(bufptr, " - state: ");
    switch (nbr->state) {
#if CONF_6LOWPAN_ND
    case NBR_GARBAGE_COLLECTIBLE:
      bufptr += sprintf(bufptr, "GARBAGE COLLECTIBLE");
      break;
    case NBR_REGISTERED:
      bufptr += sprintf(bufptr, "REGISTERED");
      break;
    case NBR_TENTATIVE:
      bufptr += sprintf(bufptr, "TENTATIVE");
      break;
    case NBR_TENTATIVE_DAD:
      bufptr += sprintf(bufptr, "TENTATIVE DAD");
      break;
#endif /* CONF_6LOWPAN_ND */
    case NBR_INCOMPLETE:
      bufptr += sprintf(bufptr, "INCOMPLETE");
      break;
    case NBR_REACHABLE:
      bufptr += sprintf(bufptr, "REACHABLE");
      break;
    case NBR_STALE:
      bufptr += sprintf(bufptr, "STALE");
      break;
    case NBR_DELAY:
      bufptr += sprintf(bufptr, "DELAY");
      break;
    case NBR_PROBE:
      bufptr += sprintf(bufptr, "PROBE");
      break;
    default:
      bufptr += sprintf(bufptr, "%d", nbr->state);
    }

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA
#if CONF_6LOWPAN_ND
    bufptr += sprintf(bufptr, " - lifetime: %lus", nbr->reachable.interval);
#else
    bufptr += sprintf(bufptr, " - reachable: %lus", nbr->reachable.interval);
#endif /* CONF_6LOWPAN_ND */
    bufptr += sprintf(bufptr, " (expiring in: %lus)", stimer_remaining(&nbr->reachable));
    nbr = nbr_table_next(ds6_neighbors, nbr);
#endif
    shell_output_str(&ifconfig_command, buf, "");
  }

  /***** Border router list *****/

#if CONF_6LOWPAN_ND
  
  sprintf(buf, "Border router list (max: %d):", UIP_DS6_BR_NB);
  shell_output_str(&ifconfig_command, buf, "");

  for(br = uip_ds6_br_list;
      br < uip_ds6_br_list + UIP_DS6_BR_NB;
      br++)
  {
    if(strlen(ipaddr_print(&br->ipaddr)) > 7)
    {
      bufptr = buf;
      bufptr += sprintf(bufptr, "    %s", ipaddr_print(&br->ipaddr));
      bufptr += sprintf(bufptr, " - version: %lu", br->version);
      bufptr += sprintf(bufptr, " - lifetime: %lus", (uint32_t)(br->lifetime == 0 ? 10000 : br->lifetime) * 60);
      bufptr += sprintf(bufptr, " (expiring in: %lus)", stimer_remaining(&br->timeout));
      shell_output_str(&ifconfig_command, buf, "");
    }
  }
  
#endif /* CONF_6LOWPAN_ND */


#if UIP_CONF_IPV6_RPL
  //printf("RPL:\n");
  //rpl_print_neighbor_list();
#endif /* CONF_6LOWPAN_ND */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void shell_ifconfig_init(void) {
  shell_register_command(&ifconfig_command);
}
/*---------------------------------------------------------------------------*/
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

