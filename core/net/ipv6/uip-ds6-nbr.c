/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
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
 *
 */

/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *    IPv6 Neighbor cache (link-layer/IPv6 address mapping)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 * \author Simon Duquennoy <simonduq@sics.se>
 *
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "lib/list.h"
#include "net/link-stats.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include "net/ipv6/uip-ds6-nbr.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#ifdef UIP_CONF_DS6_NEIGHBOR_STATE_CHANGED
#define NEIGHBOR_STATE_CHANGED(n) UIP_CONF_DS6_NEIGHBOR_STATE_CHANGED(n)
void NEIGHBOR_STATE_CHANGED(uip_ds6_nbr_t *n);
#else
#define NEIGHBOR_STATE_CHANGED(n)
#endif /* UIP_DS6_CONF_NEIGHBOR_STATE_CHANGED */

#ifdef UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK
#define LINK_NEIGHBOR_CALLBACK(addr, status, numtx) UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK(addr, status, numtx)
void LINK_NEIGHBOR_CALLBACK(const linkaddr_t *addr, int status, int numtx);
#else
#define LINK_NEIGHBOR_CALLBACK(addr, status, numtx)
#endif /* UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK */

NBR_TABLE_GLOBAL(uip_ds6_nbr_t, ds6_neighbors);

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
uip_ds6_dar_t uip_ds6_dar_list[UIP_DS6_DAR_NB]; /* \brief Duplication addresse request list */
static uip_ds6_dar_t *locdar;
extern uint32_t aro_registration_lifetime;
#endif /* CONF_6LOWPAN_ND && UIP_CONF_ROUTER */

/*---------------------------------------------------------------------------*/
void
uip_ds6_neighbors_init(void)
{
  link_stats_init();
  nbr_table_register(ds6_neighbors, (nbr_table_callback *)uip_ds6_nbr_rm);
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                uint8_t isrouter, uint8_t state, nbr_table_reason_t reason,
                void *data)
{
  uip_ds6_nbr_t *nbr = nbr_table_add_lladdr(ds6_neighbors, (linkaddr_t*)lladdr
                                            , reason, data);
  if(nbr) {
    uip_ipaddr_copy(&nbr->ipaddr, ipaddr);
#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER || CONF_6LOWPAN_ND
    nbr->isrouter = isrouter;
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */
    nbr->state = state;
#if UIP_CONF_IPV6_QUEUE_PKT
    uip_packetqueue_new(&nbr->packethandle);
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#if UIP_ND6_SEND_NA
    /* timers are set separately, for now we put them in expired state */
    stimer_set(&nbr->reachable, 0);
    stimer_set(&nbr->sendns, 0);
    nbr->nscount = 0;
#endif /* UIP_ND6_SEND_NA */
    PRINTF("Adding neighbor with ip addr ");
    PRINT6ADDR(ipaddr);
    PRINTF(" link addr ");
    PRINTLLADDR(lladdr);
    PRINTF(" state %u\n", state);
    NEIGHBOR_STATE_CHANGED(nbr);
    return nbr;
  } else {
    PRINTF("uip_ds6_nbr_add drop ip addr ");
    PRINT6ADDR(ipaddr);
    PRINTF(" link addr (%p) ", lladdr);
    PRINTLLADDR(lladdr);
    PRINTF(" state %u\n", state);
    return NULL;
  }
}

/*---------------------------------------------------------------------------*/
int
uip_ds6_nbr_rm(uip_ds6_nbr_t *nbr)
{
  if(nbr != NULL) {
#if UIP_CONF_IPV6_QUEUE_PKT
    uip_packetqueue_free(&nbr->packethandle);
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
    NEIGHBOR_STATE_CHANGED(nbr);
    return nbr_table_remove(ds6_neighbors, nbr);
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
const uip_ipaddr_t *
uip_ds6_nbr_get_ipaddr(const uip_ds6_nbr_t *nbr)
{
  return (nbr != NULL) ? &nbr->ipaddr : NULL;
}

/*---------------------------------------------------------------------------*/
const uip_lladdr_t *
uip_ds6_nbr_get_ll(const uip_ds6_nbr_t *nbr)
{
  return (const uip_lladdr_t *)nbr_table_get_lladdr(ds6_neighbors, nbr);
}
/*---------------------------------------------------------------------------*/
int
uip_ds6_nbr_num(void)
{
  uip_ds6_nbr_t *nbr;
  int num;

  num = 0;
  for(nbr = nbr_table_head(ds6_neighbors);
      nbr != NULL;
      nbr = nbr_table_next(ds6_neighbors, nbr)) {
    num++;
  }
  return num;
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_lookup(const uip_ipaddr_t *ipaddr)
{
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  if(ipaddr != NULL) {
    while(nbr != NULL) {
      if(uip_ipaddr_cmp(&nbr->ipaddr, ipaddr)) {
        return nbr;
      }
      nbr = nbr_table_next(ds6_neighbors, nbr);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_ll_lookup(const uip_lladdr_t *lladdr)
{
  return nbr_table_get_from_lladdr(ds6_neighbors, (linkaddr_t*)lladdr);
}

/*---------------------------------------------------------------------------*/
uip_ipaddr_t *
uip_ds6_nbr_ipaddr_from_lladdr(const uip_lladdr_t *lladdr)
{
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_ll_lookup(lladdr);
  return nbr ? &nbr->ipaddr : NULL;
}

/*---------------------------------------------------------------------------*/
const uip_lladdr_t *
uip_ds6_nbr_lladdr_from_ipaddr(const uip_ipaddr_t *ipaddr)
{
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(ipaddr);
  return nbr ? uip_ds6_nbr_get_ll(nbr) : NULL;
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_link_neighbor_callback(int status, int numtx)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }

  /* Update neighbor link statistics */
  link_stats_packet_sent(dest, status, numtx);
  /* Call upper-layer callback (e.g. RPL) */
  LINK_NEIGHBOR_CALLBACK(dest, status, numtx);

#if UIP_DS6_LL_NUD
  /* From RFC4861, page 72, last paragraph of section 7.3.3:
   *
   *         "In some cases, link-specific information may indicate that a path to
   *         a neighbor has failed (e.g., the resetting of a virtual circuit). In
   *         such cases, link-specific information may be used to purge Neighbor
   *         Cache entries before the Neighbor Unreachability Detection would do
   *         so. However, link-specific information MUST NOT be used to confirm
   *         the reachability of a neighbor; such information does not provide
   *         end-to-end confirmation between neighboring IP layers."
   *
   * However, we assume that receiving a link layer ack ensures the delivery
   * of the transmitted packed to the IP stack of the neighbour. This is a
   * fair assumption and allows battery powered nodes save some battery by
   * not re-testing the state of a neighbour periodically if it
   * acknowledges link packets. */
  if(status == MAC_TX_OK) {
    uip_ds6_nbr_t *nbr;
    nbr = uip_ds6_nbr_ll_lookup((uip_lladdr_t *)dest);
    if(nbr != NULL && nbr->state != NBR_INCOMPLETE) {
      nbr->state = NBR_REACHABLE;
      stimer_set(&nbr->reachable, UIP_ND6_REACHABLE_TIME / 1000);
      PRINTF("uip-ds6-neighbor : received a link layer ACK : ");
      PRINTLLADDR((uip_lladdr_t *)dest);
      PRINTF(" is reachable.\n");
    }
  }
#endif /* UIP_DS6_LL_NUD */

}
#if UIP_ND6_SEND_NA
/*---------------------------------------------------------------------------*/
/** Periodic processing on neighbors */
void
uip_ds6_neighbor_periodic(void)
{
  uip_ds6_defrt_t *locdefrt;
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
    switch(nbr->state) {
#if CONF_6LOWPAN_ND
    case NBR_GARBAGE_COLLECTIBLE:
      if(stimer_expired(&nbr->reachable)) {
        PRINTF("GARBAGE_COLLECTIBLE: remove entry (");
        PRINT6ADDR(&nbr->ipaddr);
        PRINTF(")\n");
        uip_ds6_nbr_rm(nbr);
      }
      break;
    case NBR_REGISTERED:
      if(stimer_expired(&nbr->reachable)) {
        PRINTF("REGISTERED: remove entry (");
        PRINT6ADDR(&nbr->ipaddr);
        PRINTF(")\n");
        uip_ds6_nbr_rm(nbr);
      } else if(is_timeout_percent(&nbr->reachable, UIP_DS6_NS_PERCENT_LIFETIME_RETRAN,
                                   UIP_DS6_NS_MINLIFETIME_RETRAN)) {
        PRINTF("REGISTERED: move to TENTATIVE (");
        PRINT6ADDR(&nbr->ipaddr);
        PRINTF(")\n");
        nbr->state = NBR_TENTATIVE;
        nbr->nscount = 0;
      }
      break;
    case NBR_TENTATIVE:
      if(nbr->isrouter == ISROUTER_YES) {
        if(nbr->nscount >= UIP_ND6_MAX_UNICAST_SOLICIT && 
          uip_ds6_get_global(ADDR_PREFERRED) != NULL && 
          stimer_expired(&nbr->sendns)) 
        {
          /* Need to remove the default router too. */
          if((locdefrt = uip_ds6_defrt_lookup(&nbr->ipaddr)) != NULL) {
            PRINTF("NBR periodic, TENTATIVE, NS max reached.\n");
            uip_ds6_defrt_rm(locdefrt);
          }
          uip_ds6_nbr_rm(nbr);
          PRINTF("Registration failed: remove neighbor (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
        } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
          nbr->nscount++;

          uip_ds6_addr_t *addgl = uip_ds6_get_global_br(-1, uip_ds6_defrt_lookup(&nbr->ipaddr)->br);
          if(addgl != NULL) {
            PRINTF("Sending NS (%d) with ARO with global address: ", nbr->nscount);
            PRINT6ADDR(&addgl->ipaddr);
            PRINTF("\n");
            uip_nd6_ns_output_aro(&(addgl->ipaddr), &nbr->ipaddr, &nbr->ipaddr,
                UIP_ND6_REGISTRATION_LIFETIME, 1);
          }
#if DEBUG
          else{
            PRINTF("No global address configured with the border router from the default router ");
            PRINT6ADDR(&nbr->ipaddr);
            PRINTF(".\n");
          }
#endif
#if CONF_6LOWPAN_ND_OPTI_NS
          uint16_t r = ((uint16_t)random_rand()) % ((2 << (nbr->nscount - 1) - 1) + 1);
          r = r * UIP_ND6_RTR_SOLICITATION_INTERVAL;
          if(r >= UIP_ND6_MAX_RTR_SOLICITATION_INTERVAL) {
            r = UIP_ND6_MAX_RTR_SOLICITATION_INTERVAL;
          } else if(r < uip_ds6_if.retrans_timer / 1000) {
            r = uip_ds6_if.retrans_timer / 1000;
          }
          stimer_set(&nbr->sendns, r);
#else /* CONF_6LOWPAN_ND_OPTI_NS */
          stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
#endif /* CONF_6LOWPAN_ND_OPTI_NS */
        }
      } else {
        if(stimer_expired(&nbr->reachable)) {
          uip_ds6_nbr_rm(nbr);
        }
      }
      break;

#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
    case NBR_TENTATIVE_DAD:
      // not really necessary to test if it is a router
      if( NODE_TYPE_ROUTER ){
        locdar = uip_ds6_dar_lookup_by_nbr(nbr);
        if(nbr->nscount >= UIP_ND6_MAX_UNICAST_SOLICIT) {
        uip_ds6_dar_rm(locdar);
        nbr->state = NBR_GARBAGE_COLLECTIBLE;
        } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        uip_nd6_dar_output(&uip_ds6_prefix_lookup_from_ipaddr(&locdar->ipaddr)->br->ipaddr,
                   UIP_ND6_ARO_STATUS_SUCCESS,
                   &locdar->ipaddr,
                   (uip_lladdr_t *)uip_ds6_nbr_get_ll(nbr),
                   locdar->lifetime);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
        }
      }
      break;
#endif /* UIP_CONF_ROUTER */
#endif /* CONF_6LOWPAN_ND */

    case NBR_REACHABLE:
      if(stimer_expired(&nbr->reachable)) {
#if CONF_6LOWPAN_ND && UIP_CONF_IPV6_RPL && (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
        /**
         * \sixlowpanndrpl  When RPL is activated and the node is receiving a DIO,
         *                  an NCE will be created in RECHEABLE state. As this state
         *                  must not exist for a 6lowpan-nd host and we wouldn't prevent 
         *                  the host to find new router via RPL, the host send an RS to 
         *                  add and register itself to this new router. 
         */
        if(NODE_TYPE_HOST){
          nbr->state = NBR_TENTATIVE;
          nbr->nscount = 0;
          nbr->isrouter = ISROUTER_YES;
          uip_nd6_rs_unicast_output(&nbr->ipaddr);
          tcpip_ipv6_output();
        }
#else /* CONF_6LOWPAN_ND && UIP_CONF_IPV6_RPL */
#if UIP_CONF_IPV6_RPL
        /* when a neighbor leave its REACHABLE state and is a default router,
           instead of going to STALE state it enters DELAY state in order to
           force a NUD on it. Otherwise, if there is no upward traffic, the
           node never knows if the default router is still reachable. This
           mimics the 6LoWPAN-ND behavior.
         */
        if(uip_ds6_defrt_lookup(&nbr->ipaddr) != NULL) {
          PRINTF("REACHABLE: defrt moving to DELAY (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
          nbr->state = NBR_DELAY;
          stimer_set(&nbr->reachable, UIP_ND6_DELAY_FIRST_PROBE_TIME);
          nbr->nscount = 0;
        } else {
          PRINTF("REACHABLE: moving to STALE (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
          nbr->state = NBR_STALE;
        }
#else /* UIP_CONF_IPV6_RPL */
        PRINTF("REACHABLE: moving to STALE (");
        PRINT6ADDR(&nbr->ipaddr);
        PRINTF(")\n");
        nbr->state = NBR_STALE;
#endif /* UIP_CONF_IPV6_RPL */
#endif /* CONF_6LOWPAN_ND && UIP_CONF_IPV6_RPL */
      }
      break;
    case NBR_INCOMPLETE:
      if(nbr->nscount >= UIP_ND6_MAX_MULTICAST_SOLICIT) {
        uip_ds6_nbr_rm(nbr);
      } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        PRINTF("NBR_INCOMPLETE: NS %u\n", nbr->nscount);
        uip_nd6_ns_output(NULL, NULL, &nbr->ipaddr);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
      }
      break;
    case NBR_DELAY:
      if(stimer_expired(&nbr->reachable)) {
        nbr->state = NBR_PROBE;
        nbr->nscount = 0;
        PRINTF("DELAY: moving to PROBE\n");
        stimer_set(&nbr->sendns, 0);
      }
      break;
    case NBR_PROBE:
      if(nbr->nscount >= UIP_ND6_MAX_UNICAST_SOLICIT) {
        PRINTF("PROBE END\n");
        if((locdefrt = uip_ds6_defrt_lookup(&nbr->ipaddr)) != NULL) {
          if (!locdefrt->isinfinite) {
            uip_ds6_defrt_rm(locdefrt);
          }
        }
        uip_ds6_nbr_rm(nbr);
      } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        PRINTF("PROBE: NS %u\n", nbr->nscount);
        uip_nd6_ns_output(NULL, &nbr->ipaddr, &nbr->ipaddr);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
      }
      break;
    default:
      break;
    }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_get_least_lifetime_neighbor(void)
{
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  uip_ds6_nbr_t *nbr_expiring = NULL;
  while(nbr != NULL) {
    if(nbr_expiring != NULL) {
      clock_time_t curr = stimer_remaining(&nbr->reachable);
      if(curr < stimer_remaining(&nbr->reachable)) {
        nbr_expiring = nbr;
      }
    } else {
      nbr_expiring = nbr;
    }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }
  return nbr_expiring;
}

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
uip_ds6_dar_t *
uip_ds6_dar_add(uip_ipaddr_t *ipaddr, uip_ds6_nbr_t *nbr, uint16_t lifetime)
{
  for(locdar = uip_ds6_dar_list;
      locdar < uip_ds6_dar_list + UIP_DS6_DAR_NB;
      locdar++) {
    if(locdar->nbr == NULL) {
      uip_ipaddr_copy(&locdar->ipaddr, ipaddr);
      locdar->nbr = nbr;
      locdar->lifetime = lifetime;
      return locdar;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_dar_rm(uip_ds6_dar_t *dar)
{
  dar->nbr = NULL;
}
/*---------------------------------------------------------------------------*/
uip_ds6_dar_t *
uip_ds6_dar_lookup(uip_ipaddr_t *ipaddr)
{
  for(locdar = uip_ds6_dar_list;
      locdar < uip_ds6_dar_list + UIP_DS6_DAR_NB;
      locdar++) {
    if(locdar->nbr != NULL && uip_ipaddr_cmp(&locdar->ipaddr, ipaddr)) {
      return locdar;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
uip_ds6_dar_t *
uip_ds6_dar_lookup_by_nbr(uip_ds6_nbr_t *nbr)
{
  for(locdar = uip_ds6_dar_list;
      locdar < uip_ds6_dar_list + UIP_DS6_DAR_NB;
      locdar++) {
    if(locdar->nbr != NULL && locdar->nbr == nbr) {
      return locdar;
    }
  }
  return NULL;
}
#endif /* CONF_6LOWPAN_ND && UIP_CONF_ROUTER */
#endif /* UIP_ND6_SEND_NA */
/*---------------------------------------------------------------------------*/
/** @} */
