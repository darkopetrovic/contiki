/*
 * Copyright (C) 1995, 1996, 1997, and 1998 WIDE Project.
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
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
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
 */

/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *    Neighbor discovery (RFC 4861)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 */

#include <string.h>
#include "net/ipv6/uip-icmp6.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-nameserver.h"
#include "lib/random.h"

#if UIP_CONF_IPV6_RPL
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#endif /* UIP_CONF_IPV6_RPL */

/*------------------------------------------------------------------*/
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if UIP_LOGGING
#include <stdio.h>
void uip_log(char *msg);

#define UIP_LOG(m) uip_log(m)
#else
#define UIP_LOG(m)
#endif /* UIP_LOGGING == 1 */

/*------------------------------------------------------------------*/
/** @{ */
/** \name Pointers to the header structures.
 *  All pointers except UIP_IP_BUF depend on uip_ext_len, which at
 *  packet reception, is the total length of the extension headers.
 *
 *  The pointer to ND6 options header also depends on nd6_opt_offset,
 *  which we set in each function.
 *
 *  Care should be taken when manipulating these buffers about the
 *  value of these length variables
 */

#define UIP_IP_BUF                ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])  /**< Pointer to IP header */
#define UIP_ICMP_BUF            ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])  /**< Pointer to ICMP header*/
/**@{  Pointers to messages just after icmp header */
#define UIP_ND6_RS_BUF            ((uip_nd6_rs *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_RA_BUF            ((uip_nd6_ra *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_NS_BUF            ((uip_nd6_ns *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_NA_BUF            ((uip_nd6_na *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
#define UIP_ND6_DA_BUF            ((uip_nd6_da *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#endif
/** @} */
/** Pointer to ND option */
#define UIP_ND6_OPT_HDR_BUF  ((uip_nd6_opt_hdr *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_PREFIX_BUF ((uip_nd6_opt_prefix_info *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_MTU_BUF ((uip_nd6_opt_mtu *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_RDNSS_BUF ((uip_nd6_opt_dns *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#if CONF_6LOWPAN_ND
#define UIP_ND6_OPT_ABRO_BUF ((uip_nd6_opt_abro *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_6CO_BUF ((uip_nd6_opt_6co *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#endif /* CONF_6LOWPAN_ND */
/** @} */

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
static uint8_t nd6_opt_offset;                     /** Offset from the end of the icmpv6 header to the option in uip_buf*/
static uint8_t *nd6_opt_llao;   /**  Pointer to llao option in uip_buf */
static uip_ds6_nbr_t *nbr; /**  Pointer to a nbr cache entry*/
static uip_ds6_defrt_t *defrt; /**  Pointer to a router list entry */
static uip_ds6_addr_t *addr; /**  Pointer to an interface address */
static uip_ds6_prefix_t *prefix; /**  Pointer to a prefix list entry */
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */

#if CONF_6LOWPAN_ND
static uip_nd6_opt_aro *nd6_opt_aro;   /**  Pointer to aro option in uip_buf */
#if CONF_6LOWPAN_ND_6CO
static uip_ds6_context_pref_t *context_pref;  /**  Pointer to a context prefix list entry */
static uip_nd6_opt_6co *nd6_opt_context_prefix; /**  Pointer to context 6LoWPAN context option in uip_buf */
#endif
static uip_ds6_border_router_t *border_router;  /**  Pointer to a border router list entry */
static uip_nd6_opt_abro *nd6_opt_auth_br; /**  Pointer to context authorisation border router option in uip_buf */
#if CONF_6LOWPAN_ND && CONF_6LOWPAN_ND_6CO
static uip_nd6_opt_6co *nd6_opt_6co; /**  Pointer to a 6lowpan context option in uip_buf */
#endif /* CONF_6LOWPAN_ND && CONF_6LOWPAN_ND_6CO */
#endif /* CONF_6LOWPAN_ND */

#if UIP_CONF_DYN_HOST_ROUTER
nd_node_type_t node_type;

void
set_node_type(nd_node_type_t type)
{
  uip_ipaddr_t loc_fipaddr;
  uip_ds6_maddr_t *locmaddr;
#if UIP_CONF_IPV6_RPL
  rpl_instance_t *instance;
  rpl_instance_t *end;
#endif /* UIP_CONF_IPV6_RPL */

  if(node_type != type){
    node_type = type;
    if(type == ROUTER){
      // Add the FF02::02 all router multicast address to the interface
      uip_create_linklocal_allrouters_mcast(&loc_fipaddr);
      uip_ds6_maddr_add(&loc_fipaddr);
#if UIP_CONF_IPV6_RPL
      PRINTF("Try to Reset DIO timer.\n");
      for(instance = &instance_table[0], end = instance + RPL_MAX_INSTANCES;
            instance < end; ++instance) {
          if(instance->used == 1) {
            PRINTF("Reset DIO timer.\n");
            //instance->dio_intcurrent = instance->dio_intmin+1;
            rpl_reset_dio_timer(instance);
          }
      }
#endif /* UIP_CONF_IPV6_RPL */
    } else if (type == HOST){
      // Remove FF02::02 address added previously
      uip_create_linklocal_allrouters_mcast(&loc_fipaddr);
      if((locmaddr = uip_ds6_maddr_lookup(&loc_fipaddr)) != NULL) {
        uip_ds6_maddr_rm(locmaddr);
      }
    }
  }
}
#endif /* UIP_CONF_DYN_HOST_ROUTER */

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
/*------------------------------------------------------------------*/
/* Copy link-layer address from LLAO option to a word-aligned uip_lladdr_t */
static int
extract_lladdr_from_llao_aligned(uip_lladdr_t *dest) {
  if(dest != NULL && nd6_opt_llao != NULL) {
    memcpy(dest, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], UIP_LLADDR_LEN);
    return 1;
  }
  return 0;
}
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */
/*------------------------------------------------------------------*/
/* create a llao */
static void
create_llao(uint8_t *llao, uint8_t type) {
  llao[UIP_ND6_OPT_TYPE_OFFSET] = type;
  llao[UIP_ND6_OPT_LEN_OFFSET] = UIP_ND6_OPT_LLAO_LEN >> 3;
  memcpy(&llao[UIP_ND6_OPT_DATA_OFFSET], &uip_lladdr, UIP_LLADDR_LEN);
  /* padding on some */
  memset(&llao[UIP_ND6_OPT_DATA_OFFSET + UIP_LLADDR_LEN], 0,
         UIP_ND6_OPT_LLAO_LEN - 2 - UIP_LLADDR_LEN);
}

#if CONF_6LOWPAN_ND
/* create an aro */
static void
create_aro(uint8_t *aro, uint8_t status, uint8_t lifetime, uip_lladdr_t *lladdr)
{
  ((uip_nd6_opt_aro *)aro)->type = UIP_ND6_OPT_ARO;
  ((uip_nd6_opt_aro *)aro)->len = 2;
  ((uip_nd6_opt_aro *)aro)->status = status;
  ((uip_nd6_opt_aro *)aro)->lifetime = uip_htons(lifetime);
  memcpy(&(((uip_nd6_opt_aro *)aro)->eui64), lladdr, UIP_LLADDR_LEN);
}
#endif /* CONF_6LOWPAN_ND */

/*------------------------------------------------------------------*/

#if UIP_ND6_SEND_NA
#if (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && CONF_6LOWPAN_ND
void
uip_nd6_na_output(uint8_t flags, uint8_t aro_state)
#else /* UIP_CONF_ROUTER */
void
uip_nd6_na_output(uint8_t flags)
#endif /* UIP_CONF_ROUTER */
{
  /* If the node is a router it should set R flag in NAs */
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
  if(NODE_TYPE_ROUTER){
    flags = flags | UIP_ND6_NA_FLAG_ROUTER;
  }
#endif
  uip_ext_len = 0;
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->len[0] = 0;         /* length will not be more than 255 */
  UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_NA_LEN;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;

  UIP_ICMP_BUF->type = ICMP6_NA;
  UIP_ICMP_BUF->icode = 0;

  UIP_ND6_NA_BUF->flagsreserved = flags;
  memcpy(&UIP_ND6_NA_BUF->tgtipaddr, &addr->ipaddr, sizeof(uip_ipaddr_t));

#if !CONF_6LOWPAN_ND
  create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
                    UIP_ND6_OPT_TLLAO);
  uip_len =
    UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN;
  UIP_IP_BUF->len[1] += UIP_ND6_OPT_LLAO_LEN;
#else
#if !UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
  if(NODE_TYPE_HOST){
    create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
                  UIP_ND6_OPT_TLLAO);
    uip_len =
      UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN;
    UIP_IP_BUF->len[1] += UIP_ND6_OPT_LLAO_LEN;
  } else {
    uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NA_LEN;
    if(nd6_opt_aro == NULL) {
      create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
            UIP_ND6_OPT_TLLAO);
      uip_len += UIP_ND6_OPT_LLAO_LEN;
      UIP_IP_BUF->len[1] += UIP_ND6_OPT_LLAO_LEN;
    }
  }
#else /* !UIP_CONF_ROUTER */
  uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NA_LEN;
  if(nd6_opt_aro == NULL) {
    create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
                UIP_ND6_OPT_TLLAO);
    uip_len += UIP_ND6_OPT_LLAO_LEN;
    UIP_IP_BUF->len[1] += UIP_ND6_OPT_LLAO_LEN;
  }
#endif /* !UIP_CONF_ROUTER */


#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
  if(NODE_TYPE_ROUTER){
    if(nd6_opt_aro != NULL) {
    /* Destination addr must be a local addr and derived from the EUI-64 of
     * ARO when ARO with status > 0
     */
    if(aro_state != UIP_ND6_ARO_STATUS_SUCCESS) {
      uip_create_linklocal_prefix(&UIP_IP_BUF->destipaddr);
      uip_ds6_set_addr_iid(&UIP_IP_BUF->destipaddr, (uip_lladdr_t *)&nd6_opt_aro->eui64);
    }
    /* add aro option if aro in NS is defined */
    UIP_IP_BUF->len[1] += UIP_ND6_OPT_ARO_LEN;
    create_aro(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
           aro_state, uip_ntohs(nd6_opt_aro->lifetime), (uip_lladdr_t *)&nd6_opt_aro->eui64);
    uip_len += UIP_ND6_OPT_ARO_LEN;
    }
  }
#endif /* UIP_CONF_ROUTER */
#endif /* !CONF_6LOWPAN_ND */

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending NA to [");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("] from [");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("] with target address ");
  PRINT6ADDR(&UIP_ND6_NA_BUF->tgtipaddr);
#if (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && CONF_6LOWPAN_ND
  if(NODE_TYPE_ROUTER){
  PRINTF(" with aro status:%d ", aro_state);
  }
#endif /* UIP_CONF_ROUTER */
  PRINTF("\n");

}


static void
ns_input(void)
{
  uint8_t flags;

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
  uint8_t aro_state = -1;
#endif /* CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) */

  PRINTF("Received NS from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" with target address ");
  PRINT6ADDR((uip_ipaddr_t *) (&UIP_ND6_NS_BUF->tgtipaddr));
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
  if(non_router() || NODE_TYPE_HOST) {
  PRINTF("Not yet router!\n");
    goto discard;
  }
  nd6_opt_aro = NULL;
#endif /* CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) */

#if UIP_CONF_IPV6_CHECKS
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) ||
     (uip_is_addr_mcast(&UIP_ND6_NS_BUF->tgtipaddr)) ||
     (UIP_ICMP_BUF->icode != 0)) {
    PRINTF("NS received is bad\n");
    goto discard;
  }
#endif /* UIP_CONF_IPV6_CHECKS */

  /* Options processing */
  nd6_opt_llao = NULL;
  nd6_opt_offset = UIP_ND6_NS_LEN;
  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
#if UIP_CONF_IPV6_CHECKS
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("NS received is bad\n");
      goto discard;
    }
#endif /* UIP_CONF_IPV6_CHECKS */
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_SLLAO:
      nd6_opt_llao = &uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset];
#if UIP_CONF_IPV6_CHECKS
      /* There must be NO option in a DAD NS */
      if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
        PRINTF("NS received is bad\n");
        goto discard;
      } else {
#endif /*UIP_CONF_IPV6_CHECKS */

#if CONF_6LOWPAN_ND
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
      nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
      if(nbr == NULL) {
        nbr = uip_ds6_nbr_ll_lookup((uip_lladdr_t *)&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET]);
        if(nbr == NULL) {
          printf("neighbor not found 1\n");
          goto  discard;
        }

        nbr->state = NBR_TENTATIVE_DAD;
        nbr->nscount = 0;

        if(nbr != NULL) {
          if(uip_is_addr_mcast(&UIP_IP_BUF->destipaddr)) {
            nbr->state = NBR_GARBAGE_COLLECTIBLE;
          }
          stimer_set(&nbr->reachable, UIP_ND6_TENTATIVE_NCE_LIFETIME);
          aro_state = UIP_ND6_ARO_STATUS_SUCCESS;
        } else {
          aro_state = UIP_ND6_ARO_STATUS_CACHE_FULL;
        }
      } else {
        aro_state = UIP_ND6_ARO_STATUS_SUCCESS;
      }
#endif /* UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER */
#else /* CONF_6LOWPAN_ND */
      uip_lladdr_t lladdr_aligned;
      extract_lladdr_from_llao_aligned(&lladdr_aligned);
      nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
      if(nbr == NULL) {
        uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned,
            0, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
      } else {
        const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(nbr);
        if(lladdr == NULL) {
          PRINTF("NS input, lladdr is NULL.\n");
          goto discard;
        }
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
            lladdr, UIP_LLADDR_LEN) != 0) {
          if(nbr_table_update_lladdr((const linkaddr_t *)lladdr, (const linkaddr_t *)&lladdr_aligned, 1) == 0) {
            /* failed to update the lladdr */
            PRINTF("NS input, failed to update the lladdr.\n");
            goto discard;
          }
          nbr->state = NBR_STALE;
        } else {
          if(nbr->state == NBR_INCOMPLETE) {
            nbr->state = NBR_STALE;
          }
        }
      }
#endif /* CONF_6LOWPAN_ND */
#if UIP_CONF_IPV6_CHECKS
      }
#endif /*UIP_CONF_IPV6_CHECKS */
      break;
#if CONF_6LOWPAN_ND
    case UIP_ND6_OPT_ARO:
      nd6_opt_aro = (uip_nd6_opt_aro *)UIP_ND6_OPT_HDR_BUF;
#if UIP_CONF_IPV6_CHECKS
      if(nd6_opt_aro->len != UIP_ND6_OPT_ARO_LEN / 8) {
        nd6_opt_aro = NULL;
      }
#endif
      break;
#endif /*CONF_6LOWPAN_ND*/
    default:
      PRINTF("ND option not supported in NS");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
  if(nd6_opt_llao == NULL) {
    nd6_opt_aro = NULL;
  }

  if(nd6_opt_aro) {
    nbr = uip_ds6_nbr_ll_lookup(&nd6_opt_aro->eui64);

    /* host de-registration from the router */
    if(nd6_opt_aro->lifetime == 0) {
      uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
      uip_nd6_na_output(UIP_ND6_NA_FLAG_OVERRIDE, UIP_ND6_ARO_STATUS_SUCCESS);
      tcpip_ipv6_output();
      uip_ds6_nbr_rm(nbr);
      PRINTF("ARO lifetime == 0\n");
      goto discard;
    }

    /* Process to DAD */
    if(nbr->state == NBR_TENTATIVE_DAD) {
      if(aro_state == UIP_ND6_ARO_STATUS_SUCCESS) {
        stimer_set(&nbr->reachable, UIP_ND6_MAX_RTR_SOLICITATIONS);
        uip_ds6_dar_add(&UIP_IP_BUF->srcipaddr, nbr, uip_ntohs(nd6_opt_aro->lifetime));
      }
      PRINTF("DAD - 1\n");
      goto discard;
    }
  }
#endif /* CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) */

  addr = uip_ds6_addr_lookup(&UIP_ND6_NS_BUF->tgtipaddr);
  if(addr != NULL) {
#if UIP_ND6_DEF_MAXDADNS > 0
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      /* DAD CASE */
#if UIP_CONF_IPV6_CHECKS
      if(!uip_is_addr_solicited_node(&UIP_IP_BUF->destipaddr)) {
        PRINTF("NS received is bad\n");
        goto discard;
      }
#endif /* UIP_CONF_IPV6_CHECKS */
      if(addr->state != ADDR_TENTATIVE) {
        uip_create_linklocal_allnodes_mcast(&UIP_IP_BUF->destipaddr);
        uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
        flags = UIP_ND6_NA_FLAG_OVERRIDE;
#if (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && CONF_6LOWPAN_ND
        nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr,
                              (uip_lladdr_t *)&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
                              ISROUTER_NODEFINE, NBR_GARBAGE_COLLECTIBLE);
        stimer_set(&nbr->reachable, UIP_ND6_TENTATIVE_NCE_LIFETIME);
        aro_state = UIP_ND6_ARO_STATUS_SUCCESS;
#endif /* UIP_CONF_ROUTER */
        goto create_na;
      } else {
          /** \todo if I sent a NS before him, I win */
        uip_ds6_dad_failed(addr);
        PRINTF("DAD - 2\n");
        goto discard;
      }
    }
#else /* UIP_ND6_DEF_MAXDADNS > 0 */
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      /* DAD CASE */
      PRINTF("DAD - 3\n");
      goto discard;
    }
#endif /* UIP_ND6_DEF_MAXDADNS > 0 */

#if UIP_CONF_IPV6_CHECKS
    if(uip_ds6_is_my_addr(&UIP_IP_BUF->srcipaddr)) {
        /**
         * \NOTE do we do something here? we both are using the same address.
         * If we are doing dad, we could cancel it, though we should receive a
         * NA in response of DAD NS we sent, hence DAD will fail anyway. If we
         * were not doing DAD, it means there is a duplicate in the network!
         */
      PRINTF("NS received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */

    /* Address resolution case */
    if(uip_is_addr_solicited_node(&UIP_IP_BUF->destipaddr)) {
      uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
      flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
      PRINTF("NS received: Address resolution case\n");
      goto create_na;
    }

    /* NUD CASE */
    if(uip_ds6_addr_lookup(&UIP_IP_BUF->destipaddr) == addr) {
      uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
      flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
      PRINTF("NS received: NUD CASE \n");
      goto create_na;
    } else {
#if UIP_CONF_IPV6_CHECKS
      PRINTF("NS received is bad\n");
      goto discard;
#endif /* UIP_CONF_IPV6_CHECKS */
    }
  } else {
    PRINTF("NS received, target address not mine.\n");
    goto discard;
  }


create_na:
PRINTF("NS received: create NA\n");
#if (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && CONF_6LOWPAN_ND
  uip_nd6_na_output(flags, aro_state);
#else /* (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && CONF_6LOWPAN_ND */
  uip_nd6_na_output(flags);
#endif /* UIP_CONF_ROUTER */
  return;

discard:
  uip_clear_buf();
  return;
}
#endif /* UIP_ND6_SEND_NA */


/*------------------------------------------------------------------*/
#if CONF_6LOWPAN_ND
void
uip_nd6_ns_output(uip_ipaddr_t *src, uip_ipaddr_t *dest, uip_ipaddr_t *tgt)
{
  uip_nd6_ns_output_aro(src, dest, tgt, 0, 0);
}
#endif /* CONF_6LOWPAN_ND */

void
#if !CONF_6LOWPAN_ND
uip_nd6_ns_output(uip_ipaddr_t * src, uip_ipaddr_t * dest, uip_ipaddr_t * tgt)
#else
uip_nd6_ns_output_aro(uip_ipaddr_t * src, uip_ipaddr_t * dest, uip_ipaddr_t * tgt, uint16_t lifetime, uint8_t sendaro)
#endif
{
  uip_ext_len = 0;
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;

  if(dest == NULL) {
    uip_create_solicited_node(tgt, &UIP_IP_BUF->destipaddr);
  } else {
    uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, dest);
  }
  UIP_ICMP_BUF->type = ICMP6_NS;
  UIP_ICMP_BUF->icode = 0;
  UIP_ND6_NS_BUF->reserved = 0;
  uip_ipaddr_copy((uip_ipaddr_t *) &UIP_ND6_NS_BUF->tgtipaddr, tgt);
  UIP_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
  /*
   * check if we add a SLLAO option: for DAD, MUST NOT, for NUD, MAY
   * (here yes), for Address resolution , MUST
   */
  if(!(uip_ds6_is_my_addr(tgt))) {
    if(src != NULL) {
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, src);
    } else {
      uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
    }
    if (uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      PRINTF("Dropping NS due to no suitable source address\n");
      uip_clear_buf();
      return;
    }
    UIP_IP_BUF->len[1] =
      UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN;

    create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NS_LEN],
                UIP_ND6_OPT_SLLAO);

#if CONF_6LOWPAN_ND
    if (sendaro) {
      create_aro(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN], (uint8_t)0, lifetime, &uip_lladdr);
      uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN + UIP_ND6_OPT_ARO_LEN;
  } else {
    uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN;
  }
    UIP_IP_BUF->len[1] = uip_len - UIP_IPH_LEN;
#else
    uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN;
#endif
  } else {
    uip_create_unspecified(&UIP_IP_BUF->srcipaddr);
    UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_NS_LEN;
    uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN;
  }

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending NS to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" with target address ");
  PRINT6ADDR(tgt);
  PRINTF("\n");
  return;
}
#if UIP_ND6_SEND_NA
/*------------------------------------------------------------------*/
/**
 * Neighbor Advertisement Processing
 *
 * we might have to send a pkt that had been buffered while address
 * resolution was performed (if we support buffering, see UIP_CONF_QUEUE_PKT)
 *
 * As per RFC 4861, on link layer that have addresses, TLLAO options MUST be
 * included when responding to multicast solicitations, SHOULD be included in
 * response to unicast (here we assume it is for now)
 *
 * NA can be received after sending NS for DAD, Address resolution or NUD. Can
 * be unsolicited as well.
 * It can trigger update of the state of the neighbor in the neighbor cache,
 * router in the router list.
 * If the NS was for DAD, it means DAD failed
 *
 */
static void
na_input(void)
{
#if !CONF_6LOWPAN_ND
  uint8_t is_llchange;
  uint8_t is_override;
  uip_lladdr_t lladdr_aligned;
#endif /* !CONF_6LOWPAN_ND */

#if CONF_6LOWPAN_ND && (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) || !CONF_6LOWPAN_ND
  uint8_t is_router;
#endif
  uint8_t is_solicited;

  PRINTF("Received NA from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" with target address ");
  PRINT6ADDR((uip_ipaddr_t *) (&UIP_ND6_NA_BUF->tgtipaddr));
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

  /*
   * booleans. the three last one are not 0 or 1 but 0 or 0x80, 0x40, 0x20
   * but it works. Be careful though, do not use tests such as is_router == 1
   */
#if CONF_6LOWPAN_ND && (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) || !CONF_6LOWPAN_ND
  is_router = ((UIP_ND6_NA_BUF->flagsreserved & UIP_ND6_NA_FLAG_ROUTER));
#endif
  is_solicited =
    ((UIP_ND6_NA_BUF->flagsreserved & UIP_ND6_NA_FLAG_SOLICITED));
#if !CONF_6LOWPAN_ND
  is_llchange = 0;
  is_override =
    ((UIP_ND6_NA_BUF->flagsreserved & UIP_ND6_NA_FLAG_OVERRIDE));
#endif /* !CONF_6LOWPAN_ND */

#if CONF_6LOWPAN_ND && (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
/* We are not interested in NA messages not coming from a router */
  if (!is_router && NODE_TYPE_HOST) {
  /*
   * Remove all trace in table because host use NA only with router
   * and ingore this message
   */
    /* remove entry in routing table */
    defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
    if(defrt != NULL) {
    uip_ds6_defrt_rm(defrt);
    }
    /* remove NCE if it is in */
    nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
    if(nbr != NULL) {
    uip_ds6_nbr_rm(nbr);
    }
    goto discard;
  }
#endif /* CONF_6LOWPAN_ND && !UIP_CONF_ROUTER */

#if UIP_CONF_IPV6_CHECKS
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) ||
     (UIP_ICMP_BUF->icode != 0) ||
     (uip_is_addr_mcast(&UIP_ND6_NA_BUF->tgtipaddr)) ||
     (is_solicited && uip_is_addr_mcast(&UIP_IP_BUF->destipaddr))) {
    PRINTF("NA received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  /* Options processing: we handle TLLAO, and must ignore others */
  nd6_opt_offset = UIP_ND6_NA_LEN;
  nd6_opt_llao = NULL;
#if CONF_6LOWPAN_ND
  nd6_opt_aro = NULL;
#endif
  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
#if UIP_CONF_IPV6_CHECKS
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("NA received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_TLLAO:
      nd6_opt_llao = (uint8_t *)UIP_ND6_OPT_HDR_BUF;
      break;
#if CONF_6LOWPAN_ND
    case UIP_ND6_OPT_ARO:
          nd6_opt_aro = (uip_nd6_opt_aro *)UIP_ND6_OPT_HDR_BUF;
    #if UIP_CONF_IPV6_CHECKS
          if((nd6_opt_aro->len != 2) ||
              (memcmp(&nd6_opt_aro->eui64, &uip_lladdr, UIP_LLADDR_LEN) != 0)) {
            /* ignore this option */
            nd6_opt_aro = NULL;
          }
    #endif /* UIP_CONF_IPV6_CHECKS */
      break;
#endif /* CONF_6LOWPAN_ND*/
    default:
      PRINTF("ND option not supported in NA\n");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }
  addr = uip_ds6_addr_lookup(&UIP_ND6_NA_BUF->tgtipaddr);
  /* Message processing, including TLLAO if any */
  if(addr != NULL) {
#if UIP_ND6_DEF_MAXDADNS > 0
    if(addr->state == ADDR_TENTATIVE) {
      uip_ds6_dad_failed(addr);
    }
#endif /*UIP_ND6_DEF_MAXDADNS > 0 */
    PRINTF("NA received is bad\n");
    goto discard;
  } else {
    nbr = uip_ds6_nbr_lookup(&UIP_ND6_NA_BUF->tgtipaddr);
    if(nbr == NULL) {
      goto discard;
    }

#if CONF_6LOWPAN_ND
    if(nd6_opt_aro != NULL) {
      defrt = uip_ds6_defrt_lookup(&UIP_ND6_NA_BUF->tgtipaddr);
    if (defrt != NULL) {
      if ((nd6_opt_aro->lifetime == 0)) {
        /* if lifetime is 0, that means we must remove cache entry */
         uip_ds6_nbr_rm(nbr);
         defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
         if(defrt != NULL) {
         uip_ds6_defrt_rm(defrt);
         }
      } else {
      addr = uip_ds6_addr_lookup(&UIP_IP_BUF->destipaddr);
      switch(nd6_opt_aro->status) {
      case UIP_ND6_ARO_STATUS_SUCCESS:
        addr = uip_ds6_addr_lookup(&UIP_IP_BUF->destipaddr);
        nbr->state = NBR_REGISTERED;
        nbr->nscount = 0;
        addr->state = ADDR_PREFERRED;
        stimer_set(&nbr->reachable, uip_ntohs(nd6_opt_aro->lifetime) * 60);
        break;
      case UIP_ND6_ARO_STATUS_DUPLICATE:
        /* Removing an address is done by clearing the 'isused' attribute. */
        uip_ds6_get_global_br(ADDR_TENTATIVE, defrt->br)->isused = 0;
        break;
      case UIP_ND6_ARO_STATUS_CACHE_FULL:
        /* Host SHOULD remove this router from its default router list. */
        defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
        if(defrt != NULL) {
          uip_ds6_defrt_rm(defrt);
        }
        break;
      default:
        break;
      }
      }
    }
    }

#else /* CONF_6LOWPAN_ND */

    const uip_lladdr_t *lladdr;
    lladdr = uip_ds6_nbr_get_ll(nbr);
    if(lladdr == NULL) {
      goto discard;
    }
    if(nd6_opt_llao != NULL) {
      is_llchange =
        memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], lladdr,
               UIP_LLADDR_LEN);
    }
    if(nbr->state == NBR_INCOMPLETE) {
      if(nd6_opt_llao == NULL || !extract_lladdr_from_llao_aligned(&lladdr_aligned)) {
        goto discard;
      }
      if(nbr_table_update_lladdr((const linkaddr_t *)lladdr, (const linkaddr_t *)&lladdr_aligned, 1) == 0) {
        /* failed to update the lladdr */
        goto discard;
      }

      if(is_solicited) {
        nbr->state = NBR_REACHABLE;
        nbr->nscount = 0;

        /* reachable time is stored in ms */
        stimer_set(&(nbr->reachable), uip_ds6_if.reachable_time / 1000);

      } else {
        nbr->state = NBR_STALE;
      }
      nbr->isrouter = is_router;
    } else { /* NBR is not INCOMPLETE */
      if(!is_override && is_llchange) {
        if(nbr->state == NBR_REACHABLE) {
          nbr->state = NBR_STALE;
        }
        goto discard;
      } else {
        /**
         *  If this is an cache override, or same lladdr, or no llao -
         *  do updates of nbr states.
         */
        if(is_override || !is_llchange || nd6_opt_llao == NULL) {
          if(nd6_opt_llao != NULL && is_llchange) {
            if(!extract_lladdr_from_llao_aligned(&lladdr_aligned) ||
               nbr_table_update_lladdr((const linkaddr_t *) lladdr, (const linkaddr_t *) &lladdr_aligned, 1) == 0) {
              /* failed to update the lladdr */
              goto discard;
            }
          }
          if(is_solicited) {
            nbr->state = NBR_REACHABLE;
            /* reachable time is stored in ms */
            stimer_set(&(nbr->reachable), uip_ds6_if.reachable_time / 1000);
          }
        }
      }
      if(nbr->isrouter && !is_router) {
        defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
        if(defrt != NULL) {
          uip_ds6_defrt_rm(defrt);
        }
      }
      nbr->isrouter = is_router;
    }
#endif /* !CONF_6LOWPAN_ND */
  }
#if UIP_CONF_IPV6_QUEUE_PKT
  /* The nbr is now reachable, check if we had buffered a pkt for it */
  /*if(nbr->queue_buf_len != 0) {
    uip_len = nbr->queue_buf_len;
    memcpy(UIP_IP_BUF, nbr->queue_buf, uip_len);
    nbr->queue_buf_len = 0;
    return;
    }*/
  if(uip_packetqueue_buflen(&nbr->packethandle) != 0) {
    uip_len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(UIP_IP_BUF, uip_packetqueue_buf(&nbr->packethandle), uip_len);
    uip_packetqueue_free(&nbr->packethandle);
    return;
  }

#endif /*UIP_CONF_IPV6_QUEUE_PKT */

discard:
  uip_clear_buf();
  return;
}
#endif /* UIP_ND6_SEND_NA */

#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
#if UIP_ND6_SEND_RA
/*---------------------------------------------------------------------------*/
static void
rs_input(void)
{

  PRINTF("Received RS from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

#if CONF_6LOWPAN_ND
  if(non_router() || NODE_TYPE_HOST) {
    PRINTF("RS discard because no yet router or always host.\n");
    goto discard;
  }
#endif /* CONF_6LOWPAN_ND */

#if UIP_CONF_IPV6_CHECKS
  /*
   * Check hop limit / icmp code
   * target address must not be multicast
   * if the NA is solicited, dest must not be multicast
   */
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) || (UIP_ICMP_BUF->icode != 0)) {
    PRINTF("RS received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  /* Only valid option is Source Link-Layer Address option any thing
     else is discarded */
  nd6_opt_offset = UIP_ND6_RS_LEN;
  nd6_opt_llao = NULL;

  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
#if UIP_CONF_IPV6_CHECKS
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("RS received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_SLLAO:
      nd6_opt_llao = (uint8_t *)UIP_ND6_OPT_HDR_BUF;
      break;
    default:
      PRINTF("ND option not supported in RS\n");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }
  /* Options processing: only SLLAO */
  if(nd6_opt_llao != NULL) {
#if UIP_CONF_IPV6_CHECKS
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      PRINTF("RS received is bad\n");
      goto discard;
    } else {
#endif /*UIP_CONF_IPV6_CHECKS */
#if CONF_6LOWPAN_ND
      if((nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr)) == NULL) {
        /* we need to add the neighbor */
        nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr,
                        (uip_lladdr_t *)&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
                        ISROUTER_NODEFINE, NBR_GARBAGE_COLLECTIBLE,
                        NBR_TABLE_REASON_IPV6_ND, NULL);
        stimer_set(&nbr->reachable, UIP_ND6_TENTATIVE_NCE_LIFETIME);
      } else {
        /* If LL address changed, set neighbor state to stale */
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
            uip_ds6_nbr_get_ll(nbr), UIP_LLADDR_LEN) != 0) {
          uip_ds6_nbr_t nbr_data = *nbr;
          uip_ds6_nbr_rm(nbr);
          nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr,
                                (uip_lladdr_t *)&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
                                ISROUTER_NODEFINE, NBR_GARBAGE_COLLECTIBLE,
                                NBR_TABLE_REASON_IPV6_ND, NULL);
          stimer_set(&nbr->reachable, UIP_ND6_TENTATIVE_NCE_LIFETIME);
          nbr->reachable = nbr_data.reachable;
          nbr->sendns = nbr_data.sendns;
          nbr->nscount = nbr_data.nscount;
        }
        nbr->isrouter = ISROUTER_NODEFINE;
      }
#else  /* CONF_6LOWPAN_ND */
      uip_lladdr_t lladdr_aligned;
      extract_lladdr_from_llao_aligned(&lladdr_aligned);
      if((nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr)) == NULL) {
        /* we need to add the neighbor */
        uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned,
                        0, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
      } else {
        /* If LL address changed, set neighbor state to stale */
        const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(nbr);
        if(lladdr == NULL) {
          goto discard;
        }
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
            lladdr, UIP_LLADDR_LEN) != 0) {
          uip_ds6_nbr_t nbr_data;
          nbr_data = *nbr;
          uip_ds6_nbr_rm(nbr);
          nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned,
                                0, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
          nbr->reachable = nbr_data.reachable;
          nbr->sendns = nbr_data.sendns;
          nbr->nscount = nbr_data.nscount;
        }
        nbr->isrouter = 0;
      }
#endif /* CONF_6LOWPAN_ND */
#if UIP_CONF_IPV6_CHECKS
    }
#endif /*UIP_CONF_IPV6_CHECKS */
  }

  /* Schedule a sollicited RA */
#if CONF_6LOWPAN_ND
  uip_ds6_send_ra_unicast_sollicited(&UIP_IP_BUF->srcipaddr);
#else /* CONF_6LOWPAN_ND */
  uip_ds6_send_ra_sollicited();
#endif /* CONF_6LOWPAN_ND */

discard:
  uip_clear_buf();
  return;
}

/*---------------------------------------------------------------------------*/
#if CONF_6LOWPAN_ND
void
uip_nd6_ra_output(uip_ipaddr_t * dest, uip_ds6_border_router_t *locbr)
#else
void
uip_nd6_ra_output(uip_ipaddr_t * dest)
#endif /* CONF_6LOWPAN_ND */
{
#if CONF_6LOWPAN_ND
  int len;
#endif /* UIP_CONF_6L_ROUTER */

  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;

  if(dest == NULL) {
    uip_create_linklocal_allnodes_mcast(&UIP_IP_BUF->destipaddr);
  } else {
    /* For sollicited RA */
    uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, dest);
  }
  uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);

  UIP_ICMP_BUF->type = ICMP6_RA;
  UIP_ICMP_BUF->icode = 0;

  UIP_ND6_RA_BUF->cur_ttl = uip_ds6_if.cur_hop_limit;

  UIP_ND6_RA_BUF->flags_reserved =
    (UIP_ND6_M_FLAG << 7) | (UIP_ND6_O_FLAG << 6);

#if CONF_6LOWPAN_ND
  if(uip_ds6_br_lookup(NULL) == NULL) {
    UIP_ND6_RA_BUF->flags_reserved |= 3 << 3;
  }
#endif /* CONF_6LOWPAN_ND */

  UIP_ND6_RA_BUF->router_lifetime = uip_htons(UIP_ND6_ROUTER_LIFETIME);
  //UIP_ND6_RA_BUF->reachable_time = uip_htonl(uip_ds6_if.reachable_time);
  //UIP_ND6_RA_BUF->retrans_timer = uip_htonl(uip_ds6_if.retrans_timer);
  UIP_ND6_RA_BUF->reachable_time = 0;
  UIP_ND6_RA_BUF->retrans_timer = 0;

  uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_RA_LEN;
  nd6_opt_offset = UIP_ND6_RA_LEN;


  /* Prefix list */
  for(prefix = uip_ds6_prefix_list;
      prefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB; prefix++) {
#if CONF_6LOWPAN_ND
    if((prefix->isused) && (prefix->advertise) && (locbr == prefix->br)) {
#else /* UIP_CONF_6L_ROUTER */
    if((prefix->isused) && (prefix->advertise)) {
#endif /* UIP_CONF_6L_ROUTER */
      UIP_ND6_OPT_PREFIX_BUF->type = UIP_ND6_OPT_PREFIX_INFO;
      UIP_ND6_OPT_PREFIX_BUF->len = UIP_ND6_OPT_PREFIX_INFO_LEN / 8;
      UIP_ND6_OPT_PREFIX_BUF->preflen = prefix->length;
      UIP_ND6_OPT_PREFIX_BUF->flagsreserved1 = prefix->l_a_reserved;
#if CONF_6LOWPAN_ND
      if( prefix->isinfinite ){
        UIP_ND6_OPT_PREFIX_BUF->validlt = UIP_ND6_INFINITE_LIFETIME;
      } else {
        UIP_ND6_OPT_PREFIX_BUF->validlt = uip_htonl(stimer_remaining(&prefix->vlifetime));
      }
#else /* CONF_6LOWPAN_ND */
      UIP_ND6_OPT_PREFIX_BUF->validlt = uip_htonl(prefix->vlifetime);
#endif /* CONF_6LOWPAN_ND */

      UIP_ND6_OPT_PREFIX_BUF->preferredlt = uip_htonl(prefix->plifetime);
      UIP_ND6_OPT_PREFIX_BUF->reserved2 = 0;
      uip_ipaddr_copy(&(UIP_ND6_OPT_PREFIX_BUF->prefix), &(prefix->ipaddr));
      nd6_opt_offset += UIP_ND6_OPT_PREFIX_INFO_LEN;
      uip_len += UIP_ND6_OPT_PREFIX_INFO_LEN;
    }
  }

  /* Source link-layer option */
  create_llao((uint8_t *)UIP_ND6_OPT_HDR_BUF, UIP_ND6_OPT_SLLAO);

  uip_len += UIP_ND6_OPT_LLAO_LEN;
  nd6_opt_offset += UIP_ND6_OPT_LLAO_LEN;

  /* MTU */
  UIP_ND6_OPT_MTU_BUF->type = UIP_ND6_OPT_MTU;
  UIP_ND6_OPT_MTU_BUF->len = UIP_ND6_OPT_MTU_LEN >> 3;
  UIP_ND6_OPT_MTU_BUF->reserved = 0;
  //UIP_ND6_OPT_MTU_BUF->mtu = uip_htonl(uip_ds6_if.link_mtu);
  UIP_ND6_OPT_MTU_BUF->mtu = uip_htonl(1500);

  uip_len += UIP_ND6_OPT_MTU_LEN;
  nd6_opt_offset += UIP_ND6_OPT_MTU_LEN;

#if UIP_ND6_RA_RDNSS
  if(uip_nameserver_count() > 0) {
    uint8_t i = 0;
    uip_ipaddr_t *ip = &UIP_ND6_OPT_RDNSS_BUF->ip;
    uip_ipaddr_t *dns = NULL;
    UIP_ND6_OPT_RDNSS_BUF->type = UIP_ND6_OPT_RDNSS;
    UIP_ND6_OPT_RDNSS_BUF->reserved = 0;
    UIP_ND6_OPT_RDNSS_BUF->lifetime = uip_nameserver_next_expiration();
    if(UIP_ND6_OPT_RDNSS_BUF->lifetime != UIP_NAMESERVER_INFINITE_LIFETIME) {
      UIP_ND6_OPT_RDNSS_BUF->lifetime -= clock_seconds();
    }
    while((dns = uip_nameserver_get(i)) != NULL) {
      uip_ipaddr_copy(ip++, dns);
      i++;
    }
    UIP_ND6_OPT_RDNSS_BUF->len = UIP_ND6_OPT_RDNSS_LEN + (i << 1);
    PRINTF("%d nameservers reported\n", i);
    uip_len += UIP_ND6_OPT_RDNSS_BUF->len << 3;
    nd6_opt_offset += UIP_ND6_OPT_RDNSS_BUF->len << 3;
  }
#endif /* UIP_ND6_RA_RDNSS */

  UIP_IP_BUF->len[0] = ((uip_len - UIP_IPH_LEN) >> 8);
  UIP_IP_BUF->len[1] = ((uip_len - UIP_IPH_LEN) & 0xff);

#if CONF_6LOWPAN_ND
  /* Authoritative Border Router Option */
  UIP_ND6_OPT_ABRO_BUF->type = UIP_ND6_OPT_ABRO;
  UIP_ND6_OPT_ABRO_BUF->len = UIP_ND6_OPT_ABRO_LEN / 8;
  UIP_ND6_OPT_ABRO_BUF->verlow = uip_htons(locbr->version & 0xffff);
  UIP_ND6_OPT_ABRO_BUF->verhigh = uip_htons(locbr->version >> 16);
  UIP_ND6_OPT_ABRO_BUF->lifetime = uip_htons((stimer_remaining(&locbr->timeout) / 60));
  uip_ipaddr_copy(&UIP_ND6_OPT_ABRO_BUF->address, &locbr->ipaddr);

  nd6_opt_offset += UIP_ND6_OPT_ABRO_LEN;
  uip_len += UIP_ND6_OPT_ABRO_LEN;
  UIP_IP_BUF->len[1] += UIP_ND6_OPT_ABRO_LEN;

#if CONF_6LOWPAN_ND_6CO
  /* 6LoWPAN Context Option */
  for(context_pref = uip_ds6_context_pref_list;
      context_pref < uip_ds6_context_pref_list + UIP_DS6_CONTEXT_PREF_NB;
      context_pref++) {
    if(locbr == context_pref->br && CONTEXT_PREF_USE_UNCOMPRESS(context_pref->state)) {
      len = context_pref->length < 64 ? 3 : 2;
      UIP_ND6_OPT_6CO_BUF->type = UIP_ND6_OPT_6CO;
      UIP_ND6_OPT_6CO_BUF->len = len;
      UIP_ND6_OPT_6CO_BUF->contlen = context_pref->length;
      UIP_ND6_OPT_6CO_BUF->res_c_cid = context_pref->cid |
        (CONTEXT_PREF_USE_COMPRESS(context_pref->state) ? UIP_ND6_6CO_FLAG_C : 0);
      UIP_ND6_OPT_6CO_BUF->reserved = 0x0;

      switch(context_pref->state) {
      case CONTEXT_PREF_ST_RM:
        UIP_ND6_OPT_6CO_BUF->lifetime = 0;
        break;
      case CONTEXT_PREF_ST_ADD:
        UIP_ND6_OPT_6CO_BUF->lifetime = uip_htons(context_pref->vlifetime);
        break;
      default:
        UIP_ND6_OPT_6CO_BUF->lifetime = uip_htons((stimer_remaining(&context_pref->lifetime) / 60) + 1);
        break;
      }

      uip_ipaddr_copy(&(UIP_ND6_OPT_6CO_BUF->prefix), &(context_pref->ipaddr));
      nd6_opt_offset += len * 8;
      uip_len += len * 8;
      UIP_IP_BUF->len[1] += len * 8;
    }
  }
#endif /* CONF_6LOWPAN_ND_6CO */
#endif /* CONF_6LOWPAN_ND */

  /*ICMP checksum */
  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending RA to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("\n");
  return;
}
#endif /* UIP_ND6_SEND_RA */
#endif /* UIP_CONF_ROUTER */

#if (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) || CONF_6LOWPAN_ND
/*---------------------------------------------------------------------------*/
#if CONF_6LOWPAN_ND
void
uip_nd6_rs_output(void)
{
  uip_nd6_rs_unicast_output(NULL);
}
void
uip_nd6_rs_unicast_output(uip_ipaddr_t *ipaddr)
#else /* CONF_6LOWPAN_ND */
void
uip_nd6_rs_output(void)
#endif /* CONF_6LOWPAN_ND */
{
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;
#if CONF_6LOWPAN_ND
  if (ipaddr != NULL) {
    uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, ipaddr);
  } else {
    uip_create_linklocal_allrouters_mcast(&UIP_IP_BUF->destipaddr);
  }
#else
  uip_create_linklocal_allrouters_mcast(&UIP_IP_BUF->destipaddr);
#endif /* CONF_6LOWPAN_ND */
  uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
  UIP_ICMP_BUF->type = ICMP6_RS;
  UIP_ICMP_BUF->icode = 0;
  UIP_IP_BUF->len[0] = 0;       /* length will not be more than 255 */

  if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
    UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_RS_LEN;
    uip_len = uip_l3_icmp_hdr_len + UIP_ND6_RS_LEN;
  } else {
    uip_len = uip_l3_icmp_hdr_len + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN;
    UIP_IP_BUF->len[1] =
      UIP_ICMPH_LEN + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN;

    create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_RS_LEN],
                UIP_ND6_OPT_SLLAO);
  }

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending RS to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("\n");
  return;
}
/*---------------------------------------------------------------------------*/
/**
 * Process a Router Advertisement
 *
 * - Possible actions when receiving a RA: add router to router list,
 *   recalculate reachable time, update link hop limit, update retrans timer.
 * - If MTU option: update MTU.
 * - If SLLAO option: update entry in neighbor cache
 * - If prefix option: start autoconf, add prefix to prefix list
 */
void
ra_input(void)
{
  uip_lladdr_t lladdr_aligned;
  uip_ipaddr_t ipaddr;
  uip_nd6_opt_prefix_info *nd6_opt_prefix_info;

#if CONF_6LOWPAN_ND
  nd6_opt_llao = NULL;
  nd6_opt_prefix_info = NULL;
  defrt = NULL;
  prefix = NULL;
  nbr = NULL;
  addr = NULL;
#if CONF_6LOWPAN_ND_6CO
  nd6_opt_6co = NULL;
#endif
#endif


  PRINTF("Received RA from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

#if UIP_CONF_IPV6_CHECKS
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) ||
     (!uip_is_addr_linklocal(&UIP_IP_BUF->srcipaddr)) ||
     (UIP_ICMP_BUF->icode != 0)) {
    PRINTF("RA received is bad");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

#if CONF_6LOWPAN_ND
  /* Check ABRO is present and with which version */
  PRINTF("Checking ABRO option in RA\n");
  uint32_t abro_version = 0;
  nd6_opt_auth_br = NULL;
  nd6_opt_offset = UIP_ND6_RA_LEN;

  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("RA received is bad\n");
      goto discard;
    }
    if(UIP_ND6_OPT_HDR_BUF->type == UIP_ND6_OPT_ABRO) {
      nd6_opt_auth_br = (uip_nd6_opt_abro *)UIP_ND6_OPT_HDR_BUF;
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }

  if(nd6_opt_auth_br == NULL) {
    PRINTF("RA received without ABRO\n");
    goto discard;
  }

  abro_version = uip_ntohs(nd6_opt_auth_br->verhigh);
  abro_version = uip_ntohs(nd6_opt_auth_br->verlow) + (abro_version << 16);
  border_router = uip_ds6_br_lookup(&nd6_opt_auth_br->address);
  if(border_router != NULL && (abro_version < (border_router->version))) {
    PRINTF("RA received with lower ABRO version\n");
    goto discard;
  }

  if(border_router == NULL) {
    /* New border router found */
    border_router = uip_ds6_br_add(abro_version, nd6_opt_auth_br->lifetime,
                                   &nd6_opt_auth_br->address);
    PRINTF("New border router (");
    PRINT6ADDR(&nd6_opt_auth_br->address);
    PRINTF(") with ABRO version %lu\n", abro_version);
  }

  abro_version -= border_router->version;
  if(abro_version > 0) {
    /* New version, so remove all prefix and context */
    uip_ds6_prefix_rm_all(border_router);
    border_router->version += abro_version;
  }
#endif /* CONF_6LOWPAN_ND */

  if(UIP_ND6_RA_BUF->cur_ttl != 0) {
    uip_ds6_if.cur_hop_limit = UIP_ND6_RA_BUF->cur_ttl;
    PRINTF("uip_ds6_if.cur_hop_limit %u\n", uip_ds6_if.cur_hop_limit);
  }

  if(UIP_ND6_RA_BUF->reachable_time != 0) {
    if(uip_ds6_if.base_reachable_time !=
       uip_ntohl(UIP_ND6_RA_BUF->reachable_time)) {
      uip_ds6_if.base_reachable_time = uip_ntohl(UIP_ND6_RA_BUF->reachable_time);
      uip_ds6_if.reachable_time = uip_ds6_compute_reachable_time();
    }
  }
  if(UIP_ND6_RA_BUF->retrans_timer != 0) {
    uip_ds6_if.retrans_timer = uip_ntohl(UIP_ND6_RA_BUF->retrans_timer);
  }

  /* Options processing */
  nd6_opt_offset = UIP_ND6_RA_LEN;
  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("RA received is bad");
      goto discard;
    }
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_SLLAO:
      PRINTF("Processing SLLAO option in RA\n");
      nd6_opt_llao = (uint8_t *) UIP_ND6_OPT_HDR_BUF;
      nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
      if(!extract_lladdr_from_llao_aligned(&lladdr_aligned)) {
        /* failed to extract llao - discard packet */
        goto discard;
      }
#if CONF_6LOWPAN_ND
    if(nbr == NULL) {
      nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr,
                  (uip_lladdr_t *)&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
                  ISROUTER_YES, NBR_TENTATIVE, NBR_TABLE_REASON_IPV6_ND, NULL);
      stimer_set(&nbr->reachable, UIP_ND6_TENTATIVE_NCE_LIFETIME);
    }
#else /* CONF_6LOWPAN_ND */
      if(nbr == NULL) {
        nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned,
                              1, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
      } else {
        const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(nbr);
        if(lladdr == NULL) {
          goto discard;
        }
        if(nbr->state == NBR_INCOMPLETE) {
          nbr->state = NBR_STALE;
        }
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
                  lladdr, UIP_LLADDR_LEN) != 0) {
          /* change of link layer address */
          if(nbr_table_update_lladdr((const linkaddr_t *)lladdr, (const linkaddr_t *)&lladdr_aligned, 1) == 0) {
            /* failed to update the lladdr */
            goto discard;
          }
          nbr->state = NBR_STALE;
        }
        nbr->isrouter = 1;
      }
#endif /* CONF_6LOWPAN_ND */
      break;
    case UIP_ND6_OPT_MTU:
      PRINTF("Processing MTU option in RA\n");
      uip_ds6_if.link_mtu =
        uip_ntohl(((uip_nd6_opt_mtu *) UIP_ND6_OPT_HDR_BUF)->mtu);
      break;
    case UIP_ND6_OPT_PREFIX_INFO:
      PRINTF("Processing PREFIX option in RA\n");
      nd6_opt_prefix_info = (uip_nd6_opt_prefix_info *) UIP_ND6_OPT_HDR_BUF;
      if((uip_ntohl(nd6_opt_prefix_info->validlt) >=
          uip_ntohl(nd6_opt_prefix_info->preferredlt))
         && (!uip_is_addr_linklocal(&nd6_opt_prefix_info->prefix))) {
        /* on-link flag related processing */
#if CONF_6LOWPAN_ND && 0
        // myFIXME: remove On-link flag in the RA message before activating this
        if(!(nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_ONLINK)) {
        /* I.D.ietf-6lowpan-nd section 5.4: Should the host erroneously receive
       * a Prefix Information option with the 'L' (on-link) flag set, then that
       * Prefix Information Option (PIO) MUST be ignored. */

#else
        if(nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_ONLINK) {
#endif
          prefix =
            uip_ds6_prefix_lookup(&nd6_opt_prefix_info->prefix,
                                  nd6_opt_prefix_info->preflen);
          if(prefix == NULL) {
            if(nd6_opt_prefix_info->validlt != 0) {
#if UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER
              if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
                prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                            nd6_opt_prefix_info->preflen,
                            1, nd6_opt_prefix_info->flagsreserved1,
                            uip_ntohl(nd6_opt_prefix_info->
                                  validlt),
                            uip_ntohl(nd6_opt_prefix_info->
                                  preferredlt));
              } else {
                prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                            nd6_opt_prefix_info->preflen,
                            1, nd6_opt_prefix_info->flagsreserved1,
                            0, 0);
              }
#else /* UIP_CONF_ROUTER */
              if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
                prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                                            nd6_opt_prefix_info->preflen,
                                            uip_ntohl(nd6_opt_prefix_info->
                                                  validlt));
              } else {
                prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                                            nd6_opt_prefix_info->preflen, 0);
              }
#endif /* UIP_CONF_ROUTER */
#if CONF_6LOWPAN_ND
              prefix->br = border_router;
#endif /* CONF_6LOWPAN_ND */
            }
          } else {
            switch (nd6_opt_prefix_info->validlt) {
            case 0:
              uip_ds6_prefix_rm(prefix);
              break;
            case UIP_ND6_INFINITE_LIFETIME:
              prefix->isinfinite = 1;
              break;
            default:
              PRINTF("Updating timer of prefix ");
              PRINT6ADDR(&prefix->ipaddr);
              PRINTF(" new value %lu\n", uip_ntohl(nd6_opt_prefix_info->validlt));
              stimer_set(&prefix->vlifetime,
                         uip_ntohl(nd6_opt_prefix_info->validlt));
              prefix->isinfinite = 0;
              break;
            }
          }
        }
        /* End of on-link flag related processing */
        /* autonomous flag related processing */
        if((nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_AUTONOMOUS)
           && (nd6_opt_prefix_info->validlt != 0)
           && (nd6_opt_prefix_info->preflen == UIP_DEFAULT_PREFIX_LEN)) {

          uip_ipaddr_copy(&ipaddr, &nd6_opt_prefix_info->prefix);
          uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
          addr = uip_ds6_addr_lookup(&ipaddr);
          if((addr != NULL) && (addr->type == ADDR_AUTOCONF)) {
            if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
              /* The processing below is defined in RFC4862 section 5.5.3 e */
              if((uip_ntohl(nd6_opt_prefix_info->validlt) > 2 * 60 * 60) ||
                 (uip_ntohl(nd6_opt_prefix_info->validlt) >
                  stimer_remaining(&addr->vlifetime))) {
                PRINTF("Updating timer of address ");
                PRINT6ADDR(&addr->ipaddr);
                PRINTF(" new value %lu\n",
                       uip_ntohl(nd6_opt_prefix_info->validlt));
                stimer_set(&addr->vlifetime,
                           uip_ntohl(nd6_opt_prefix_info->validlt));
              } else {
                stimer_set(&addr->vlifetime, 2 * 60 * 60);
                PRINTF("Updating timer of address ");
                PRINT6ADDR(&addr->ipaddr);
                PRINTF(" new value %lu\n", (unsigned long)(2 * 60 * 60));
              }
              addr->isinfinite = 0;
            } else {
              addr->isinfinite = 1;
            }
          } else {
            /* NOTE: Normally the lifetime of the address must be the
             * Prefeered Lifetime specified in the RA.
             * But in this case, even if the preferred lifetime expires
             * and the address become 'deprecated', it can still be used.
             */
            if(uip_ntohl(nd6_opt_prefix_info->validlt) ==
               UIP_ND6_INFINITE_LIFETIME) {
              uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
            } else {
              uip_ds6_addr_add(&ipaddr, uip_ntohl(nd6_opt_prefix_info->validlt),
                               ADDR_AUTOCONF);
            }
          }
        }
        /* End of autonomous flag related processing */
      }
      break;
#if UIP_ND6_RA_RDNSS
    case UIP_ND6_OPT_RDNSS:
      if(UIP_ND6_RA_BUF->flags_reserved & (UIP_ND6_O_FLAG << 6)) {
        PRINTF("Processing RDNSS option\n");
        uint8_t naddr = (UIP_ND6_OPT_RDNSS_BUF->len - 1) / 2;
        uip_ipaddr_t *ip = (uip_ipaddr_t *)(&UIP_ND6_OPT_RDNSS_BUF->ip);
        PRINTF("got %d nameservers\n", naddr);
        while(naddr-- > 0) {
          PRINTF(" nameserver: ");
          PRINT6ADDR(ip);
          PRINTF(" lifetime: %lx\n", uip_ntohl(UIP_ND6_OPT_RDNSS_BUF->lifetime));
          uip_nameserver_update(ip, uip_ntohl(UIP_ND6_OPT_RDNSS_BUF->lifetime));
          ip++;
        }
      }
      break;
#endif /* UIP_ND6_RA_RDNSS */
#if CONF_6LOWPAN_ND
#if CONF_6LOWPAN_ND_6CO

  case UIP_ND6_OPT_6CO:
  PRINTF("Processing 6CO option in RA\n");
  nd6_opt_context_prefix = (uip_nd6_opt_6co *)UIP_ND6_OPT_HDR_BUF;
    context_pref = uip_ds6_context_pref_lookup_by_cid(
      nd6_opt_context_prefix->res_c_cid & UIP_ND6_6CO_FLAG_CID);
    if(context_pref == NULL) {
    /* New entry must in context prefix table */
    if(nd6_opt_context_prefix->lifetime != 0) {
      context_pref = uip_ds6_context_pref_add(&nd6_opt_context_prefix->prefix,
                          nd6_opt_context_prefix->contlen,
                          nd6_opt_context_prefix->res_c_cid & (UIP_ND6_6CO_FLAG_C | UIP_ND6_6CO_FLAG_CID),
                          uip_ntohs(nd6_opt_context_prefix->lifetime), UIP_ND6_RA_BUF->router_lifetime);
      context_pref->br = border_router;
    }
    } else if(context_pref->br == border_router) {
    /* Update entry already in table */
    if(nd6_opt_context_prefix->lifetime == 0) {
      /* context entry MUST be removed immediately */
      uip_ds6_context_pref_rm(context_pref);
    } else {
      /* update lifetime */
      if(nd6_opt_context_prefix->lifetime != 0 && context_pref->state != CONTEXT_PREF_ST_ADD) {
      context_pref->state = nd6_opt_context_prefix->res_c_cid & UIP_ND6_6CO_FLAG_C ?
        CONTEXT_PREF_ST_COMPRESS : CONTEXT_PREF_ST_UNCOMPRESSONLY;
      stimer_set(&context_pref->lifetime, uip_ntohs(nd6_opt_context_prefix->lifetime) * 60);
      }
      PRINTF("Updating timer of prefix ");
      PRINT6ADDR(&context_pref->ipaddr);
      PRINTF("/%d \n", nd6_opt_context_prefix->len);
    }
    }
    break;
#endif /* CONF_6LOWPAN_ND_6CO */
  case UIP_ND6_OPT_ABRO:
    /* Process ABRO option */
  PRINTF("Processing ABRO option in RA\n");
  if(abro_version >= 0) {
    /* Update timer */
    stimer_set(&border_router->timeout,
         (nd6_opt_auth_br->lifetime == 0 ? 10000 : nd6_opt_auth_br->lifetime) * 60);
  }
  if(abro_version > 0) {
    /* Update information */
    border_router->version = uip_ntohs(nd6_opt_auth_br->verhigh);
    border_router->version = uip_ntohs(nd6_opt_auth_br->verlow) + (border_router->version << 16);
    uip_ipaddr_copy(&border_router->ipaddr, &nd6_opt_auth_br->address);
  }
    break;

#endif /* CONF_6LOWPAN_ND */
    default:
      PRINTF("ND option not supported in RA");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }

  defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
  if(UIP_ND6_RA_BUF->router_lifetime != 0) {
    if(nbr != NULL) {
      nbr->isrouter = 1;
    }
    if(defrt == NULL) {
      defrt = uip_ds6_defrt_add(&UIP_IP_BUF->srcipaddr,
                        (unsigned
                         long)(uip_ntohs(UIP_ND6_RA_BUF->router_lifetime)));
    } else {
      stimer_set(&(defrt->lifetime),
                 (unsigned long)(uip_ntohs(UIP_ND6_RA_BUF->router_lifetime)));
    }
  } else {
    if(defrt != NULL) {
      uip_ds6_defrt_rm(defrt);
    }
  }

#if CONF_6LOWPAN_ND
  if(defrt == NULL) {
    PRINTF("RA input: default router lifetime set to 0. Neighbor set as garbage collectible.\n");
    nbr->state = NBR_GARBAGE_COLLECTIBLE;
    goto discard;
  } else {
    defrt->br = border_router;
    defrt->state = DEFRT_ST_RA_RCV;
  }
#endif /* CONF_6LOWPAN_ND */

#if UIP_CONF_IPV6_QUEUE_PKT
  /* If the nbr just became reachable (e.g. it was in NBR_INCOMPLETE state
   * and we got a SLLAO), check if we had buffered a pkt for it */
  /*  if((nbr != NULL) && (nbr->queue_buf_len != 0)) {
    uip_len = nbr->queue_buf_len;
    memcpy(UIP_IP_BUF, nbr->queue_buf, uip_len);
    nbr->queue_buf_len = 0;
    return;
    }*/
  if(nbr != NULL && uip_packetqueue_buflen(&nbr->packethandle) != 0) {
    uip_len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(UIP_IP_BUF, uip_packetqueue_buf(&nbr->packethandle), uip_len);
    uip_packetqueue_free(&nbr->packethandle);
    return;
  }

#endif /*UIP_CONF_IPV6_QUEUE_PKT */

discard:
  uip_clear_buf();
  return;
}
#endif /* !UIP_CONF_ROUTER */

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
void
uip_nd6_da_output(uip_ipaddr_t *destipaddr, uint8_t type, uint8_t status,
                  uip_ipaddr_t *hostipaddr, uip_lladdr_t *eui64, uint16_t lifetime)
{
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;
  UIP_ICMP_BUF->type = type;
  UIP_ICMP_BUF->icode = 0;
  UIP_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
  UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_DA_LEN;
  uip_len = uip_l3_icmp_hdr_len + UIP_ND6_DA_LEN;

  UIP_ND6_DA_BUF->status = status;
  UIP_ND6_DA_BUF->reserved = 0;
  UIP_ND6_DA_BUF->lifetime = uip_htons(lifetime);
  memcpy(&UIP_ND6_DA_BUF->eui64, eui64, UIP_LLADDR_LEN);
  uip_ipaddr_copy(&UIP_ND6_DA_BUF->regipaddr, hostipaddr);

  uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, destipaddr);
  uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr,
                  &uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending %s to ", type == ICMP6_DAR ? "DAR" : "DAC");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" with host address ");
  PRINT6ADDR(&UIP_ND6_DA_BUF->regipaddr);
  PRINTF(" with status %d\n", status);

  return;
}

/*---------------------------------------------------------------------------*/
void
uip_nd6_dar_output(uip_ipaddr_t *destipaddr, uint8_t status,
                   uip_ipaddr_t *hostipaddr, uip_lladdr_t *eui64, uint16_t lifetime)
{
  uip_nd6_da_output(destipaddr, ICMP6_DAR, status, hostipaddr, eui64, lifetime);
}
/*---------------------------------------------------------------------------*/
/**
 *
 * \brief process a Duplication Address Confirmation
 *
 * - When receiving a DAC, we add the entry on the Neighbor Cache is it
 *   was a success. We send back to the host a NA to notify it of the
 *   decision.
 */
void
dac_input(void)
{
  uint8_t aro_state;
  static uip_nd6_opt_aro aro;
  static uip_ds6_dar_t *dar;

  PRINTF("Received DAC from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" with host address ");
  PRINT6ADDR((uip_ipaddr_t *)(&UIP_ND6_DA_BUF->regipaddr));
  PRINTF("\n");

  UIP_STAT(++uip_stat.nd6.recv);

#if UIP_CONF_IPV6_CHECKS
  if((UIP_ICMP_BUF->icode != 0) ||
     (UIP_IP_BUF->len[1] < 32) ||
     (uip_is_addr_mcast(&UIP_ND6_DA_BUF->regipaddr)) ||
     (uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr))) {
    PRINTF("DAR received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  nbr = uip_ds6_nbr_ll_lookup(&UIP_ND6_DA_BUF->eui64);
  if(nbr == NULL ||
     !(dar = uip_ds6_dar_lookup_by_nbr(nbr)) ||
     !uip_ipaddr_cmp(&dar->ipaddr, &UIP_ND6_DA_BUF->regipaddr)) {
    /* No in NCE, so silently ignored */
    goto discard;
  } else if(UIP_ND6_DA_BUF->status == UIP_ND6_ARO_STATUS_SUCCESS) {
    nbr->state = NBR_REGISTERED;
    stimer_set(&nbr->reachable, uip_ntohs(UIP_ND6_DA_BUF->lifetime) * 60);
    uip_ds6_route_add(&dar->ipaddr, 128, &nbr->ipaddr);
    aro_state = UIP_ND6_ARO_STATUS_SUCCESS;
  } else {
    aro_state = UIP_ND6_DA_BUF->status;
  }

  /* send na */
  nd6_opt_aro = &aro;
  nd6_opt_aro->lifetime = UIP_ND6_DA_BUF->lifetime;
  nd6_opt_aro->status = UIP_ND6_DA_BUF->status;
  memcpy(&nd6_opt_aro->eui64, &UIP_ND6_DA_BUF->eui64, UIP_LLADDR_LEN);
  uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &dar->ipaddr);
  addr = uip_ds6_get_link_local(ADDR_PREFERRED);
  uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &addr->ipaddr);
  uip_nd6_na_output(UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE,
                    aro_state);

  /* remove all entries */
  uip_ds6_dar_rm(dar);
  if(aro_state != UIP_ND6_ARO_STATUS_SUCCESS) {
    tcpip_ipv6_output(); /* force to send before remove NCE */
    uip_ds6_nbr_rm(nbr);
  }
  return;

discard:
  uip_len = 0;
  return;
}
#endif /* CONF_6LOWPAN_ND && UIP_CONF_ROUTER */

/*------------------------------------------------------------------*/
/* ICMPv6 input handlers */
#if UIP_ND6_SEND_NA
#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
UIP_ICMP6_HANDLER(ns_input_handler, ICMP6_NS, UIP_ICMP6_HANDLER_CODE_ANY,
                  ns_input);
#else
UIP_ICMP6_HANDLER(ns_input_handler, ICMP6_NS, UIP_ICMP6_HANDLER_CODE_ANY,
                  ns_input);
#endif /* #if CONF_6LOWPAN_ND && UIP_CONF_ROUTER*/
UIP_ICMP6_HANDLER(na_input_handler, ICMP6_NA, UIP_ICMP6_HANDLER_CODE_ANY,
                  na_input);
#endif

#if (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && UIP_ND6_SEND_RA
UIP_ICMP6_HANDLER(rs_input_handler, ICMP6_RS, UIP_ICMP6_HANDLER_CODE_ANY,
                  rs_input);
#endif

#if (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) || CONF_6LOWPAN_ND
UIP_ICMP6_HANDLER(ra_input_handler, ICMP6_RA, UIP_ICMP6_HANDLER_CODE_ANY,
                  ra_input);
#endif

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
UIP_ICMP6_HANDLER(dac_input_handler, ICMP6_DAC, UIP_ICMP6_HANDLER_CODE_ANY,
                  dac_input);
#endif /* UIP_CONF_6LR */
/*---------------------------------------------------------------------------*/
void
uip_nd6_init()
{

#if UIP_ND6_SEND_NA
#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
  /* Only handle NSs if we are prepared to send out NAs */
  uip_icmp6_register_input_handler(&ns_input_handler);
#else
  uip_icmp6_register_input_handler(&ns_input_handler);
#endif /* #if CONF_6LOWPAN_ND && UIP_CONF_ROUTER */

  /*
   * Only handle NAs if we are prepared to send out NAs.
   * This is perhaps logically incorrect, but this condition was present in
   * uip_process and we keep it until proven wrong
   */
  uip_icmp6_register_input_handler(&na_input_handler);
#endif /* UIP_ND6_SEND_NA */

#if (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) && UIP_ND6_SEND_RA
  /* Only accept RS if we are a router and happy to send out RAs */
  uip_icmp6_register_input_handler(&rs_input_handler);
#endif

#if (!UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER) || CONF_6LOWPAN_ND
  /* Only process RAs if we are not a router */
  /* Process in any case when 6lowpan optimization is used */
  uip_icmp6_register_input_handler(&ra_input_handler);
#endif

#if CONF_6LOWPAN_ND && (UIP_CONF_ROUTER || UIP_CONF_DYN_HOST_ROUTER)
  /* Only process DACs if we are not a 6LoWPAN-ND router */
  uip_icmp6_register_input_handler(&dac_input_handler);
#endif /* CONF_6LOWPAN_ND && UIP_CONF_ROUTER  */

#if UIP_CONF_DYN_HOST_ROUTER
  node_type = HOST;
#endif /* UIP_CONF_DYN_HOST_ROUTER */
}
/*---------------------------------------------------------------------------*/
 /** @} */
