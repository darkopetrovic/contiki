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

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "contiki.h"
#include "shell-ifconfig.h"

#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
PROCESS(shell_ifconfig_process, "ifconfig");
SHELL_COMMAND(ifconfig_command, "ifconfig",
    "ifconfig : show interface configuration", &shell_ifconfig_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(shell_ifconfig_process, ev, data) {
  uint8_t i;
  uip_ds6_defrt_t *d;
  uip_ds6_element_t *element;
  uip_ds6_nbr_t *nbr;
  uip_ds6_prefix_t *prefix;
#if CONF_6LOWPAN_ND
  uip_ds6_border_router_t *br;
#endif

  PROCESS_BEGIN();

  printf("Interface configuration:\n");
  printf("    Link MTU: %lu\n", uip_ds6_if.link_mtu);
  printf("    Hop limit: %d\n", uip_ds6_if.cur_hop_limit);
  printf("    Base reachable time: %lums\n", uip_ds6_if.base_reachable_time);
  printf("    Reachable time: %lums\n", uip_ds6_if.reachable_time);
  printf("    Retransmission timer: %lums\n", uip_ds6_if.retrans_timer);
  printf("    Max DADNS: %d\n", uip_ds6_if.maxdadns);

  printf("MAC Address:\n    ");
  for (i = 0; i < LINKADDR_SIZE - 1; i++) {
    printf("%02x:", linkaddr_node_addr.u8[i]);
  }
  printf("%02x\n", linkaddr_node_addr.u8[i]);

  printf("Prefixes list (max %d):\n", UIP_DS6_PREFIX_NB);
  for (prefix = uip_ds6_prefix_list;
      prefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB; prefix++) {
    if (prefix->isused) {
      printf("    ");
      uip_debug_ipaddr_print(&prefix->ipaddr);
      printf("/%d", prefix->length);

      printf(" - lifetime: ");

      if( prefix->isinfinite ) {
        printf("infinite");
      } else {
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER || CONF_6LOWPAN_ND
#if CONF_6LOWPAN_ND
        printf("%lus", prefix->vlifetime.interval);
        printf(" (expiring in: %lus)", stimer_remaining(&prefix->vlifetime));
#else /* CONF_6LOWPAN_ND */
        printf("%lus", prefix->vlifetime);
#endif /* CONF_6LOWPAN_ND */
#else /* UIP_CONF_ROUTER */
        printf("%lus", prefix->vlifetime.interval);
        printf(" (expiring in: %lus)", stimer_remaining(&prefix->vlifetime));
#endif /* UIP_CONF_ROUTER */
      }
      printf("\n");
    }
  }

  printf("Addresses list (max: %d):\n", UIP_DS6_ADDR_NB);
  for (element = (uip_ds6_element_t *) uip_ds6_if.addr_list;
      element
          < (uip_ds6_element_t *) ((uint8_t *) (uip_ds6_element_t *) uip_ds6_if.addr_list
              + (UIP_DS6_ADDR_NB * sizeof(uip_ds6_addr_t)));
      element = (uip_ds6_element_t *) ((uint8_t *) element
          + sizeof(uip_ds6_addr_t))) {
    if (element->isused) {

      uip_ds6_addr_t *locaddr = (uip_ds6_addr_t *) element;
      printf("    ");
      uip_debug_ipaddr_print(&element->ipaddr);

      printf(" - state: ");
      switch (locaddr->state) {
      case ADDR_TENTATIVE:
        printf("TENTATIVE");
        break;
      case ADDR_PREFERRED:
        printf("PREFERRED");
        break;
      case ADDR_DEPRECATED:
        printf("DEPRECATED");
        break;
      }

      printf(" - type: ");
      switch (locaddr->type) {
      case ADDR_AUTOCONF:
        printf("Auto");
        break;
      case ADDR_DHCP:
        printf("DHCP");
        break;
      case ADDR_MANUAL:
        printf("Manual");
        break;
      }

      printf(" - lifetime: ");
      if (locaddr->isinfinite) {
        printf("infinite");
      } else {
        printf("%lus", locaddr->vlifetime.interval);
        printf(" (expiring in: %lus)", stimer_remaining(&locaddr->vlifetime));
      }

      printf("\n");

    }
  }

  printf("Default routers (max: %d):\n", UIP_DS6_DEFRT_NB);
  for (d = uip_ds6_defrt_list_head(); d != NULL; d = list_item_next(d)) {
    printf("   ");
    uip_debug_ipaddr_print(&d->ipaddr);
    printf(" - lifetime: ");
    if (d->isinfinite) {
      printf("infinite");
    } else {
      printf("%lus", d->lifetime.interval);
      printf(" (expiring in: %lus)", stimer_remaining(&d->lifetime));
    }
    printf("\n");
  }

  printf("Neighbors list (max: %d):\n", NBR_TABLE_MAX_NEIGHBORS);
  nbr = nbr_table_head(ds6_neighbors);
  while (nbr != NULL) {
    printf("   ");
    uip_debug_ipaddr_print(&nbr->ipaddr);
    printf(" - router: ");
    if (nbr->isrouter) {
      printf("YES");
    } else {
      printf("NO");
    }

    printf(" - state: ");
    switch (nbr->state) {
#if CONF_6LOWPAN_ND
    case NBR_GARBAGE_COLLECTIBLE:
      printf("GARBAGE COLLECTIBLE");
      break;
    case NBR_REGISTERED:
      printf("REGISTERED");
      break;
    case NBR_TENTATIVE:
      printf("TENTATIVE");
      break;
    case NBR_TENTATIVE_DAD:
      printf("TENTATIVE DAD");
      break;
#else /* CONF_6LOWPAN_ND */
    case NBR_INCOMPLETE:
      printf("INCOMPLETE");
      break;
    case NBR_REACHABLE:
      printf("REACHABLE");
      break;
    case NBR_STALE:
      printf("STALE");
      break;
    case NBR_DELAY:
      printf("DELAY");
      break;
    case NBR_PROBE:
      printf("PROBE");
      break;
#endif /* CONF_6LOWPAN_ND */
    default:
      printf("%d", nbr->state);
    }

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA
#if CONF_6LOWPAN_ND
    printf(" - lifetime: %lus", nbr->reachable.interval);
#else
    printf(" - reachable: %lus", nbr->reachable.interval);
#endif /* CONF_6LOWPAN_ND */
    printf(" (expiring in: %lus)", stimer_remaining(&nbr->reachable));
    printf("\n");
    nbr = nbr_table_next(ds6_neighbors, nbr);
#endif
  }

#if CONF_6LOWPAN_ND
  printf("Border router list (max: %d):\n", UIP_DS6_BR_NB);
  for(br = uip_ds6_br_list;
      br < uip_ds6_br_list + UIP_DS6_BR_NB;
      br++)
  {
    printf("    ");
    uip_debug_ipaddr_print(&br->ipaddr);
    printf(" - version: %lu", br->version);
    printf(" - lifetime: %lus", (uint32_t)((br->lifetime == 0 ? 10000 : br->lifetime) * 60));
    printf(" (expiring in: %lus)\n", stimer_remaining(&br->timeout));
  }
#endif /* CONF_6LOWPAN_ND */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void shell_ifconfig_init(void) {
  shell_register_command(&ifconfig_command);
}
/*---------------------------------------------------------------------------*/
