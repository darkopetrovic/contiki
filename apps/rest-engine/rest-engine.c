/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
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
 */

/**
 * \file
 *      An abstraction layer for RESTful Web services (Erbium).
 *      Inspired by RESTful Contiki by Dogan Yazar.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <string.h>
#include <stdio.h>
#include "contiki.h"
#include "rest-engine.h"
#include "er-coap-observe.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#if REST_DELAY_RES_START
struct etimer observer_periodic_timer;
static struct process *observer_periodic_timer_p;
#endif

PROCESS(rest_engine_process, "REST Engine");
/*---------------------------------------------------------------------------*/
LIST(restful_services);
LIST(restful_periodic_services);
/*---------------------------------------------------------------------------*/
/*- REST Engine API ---------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * \brief Initializes and starts the REST Engine process
 *
 * This function must be called by server processes before any resources are
 * registered through rest_activate_resource().
 */
void
rest_init_engine(void)
{
  /* avoid initializing twice */
  static uint8_t initialized = 0;

  if(initialized) {
    PRINTF("REST engine process already running - double initialization?\n");
    return;
  }
  initialized = 1;

  list_init(restful_services);

  REST.set_service_callback(rest_invoke_restful_service);

  /* Start the RESTful server implementation. */
  REST.init();

  /*Start REST engine process */
  process_start(&rest_engine_process, NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Makes a resource available under the given URI path
 * \param resource A pointer to a resource implementation
 * \param path The URI path string for this resource
 *
 * The resource implementation must be imported first using the
 * extern keyword. The build system takes care of compiling every
 * *.c file in the ./resources/ sub-directory (see example Makefile).
 */
void
rest_activate_resource(resource_t *resource, char *path)
{
  resource->url = path;
  list_add(restful_services, resource);

  PRINTF("Activating: %s\n", resource->url);

  /* Add ALL periodic resources to be present in the list. They are activated later. */
#if REST_DELAY_RES_START
  if(resource->flags & IS_PERIODIC && resource->periodic->periodic_handler)
#else
   /* Only add periodic resources with a periodic_handler and a period > 0. */
   if(resource->flags & IS_PERIODIC && resource->periodic->periodic_handler
      && resource->periodic->period)
#endif
  {
    PRINTF("Periodic resource: %p (%s)\n", resource->periodic,
           resource->periodic->resource->url);
    list_add(restful_periodic_services, resource->periodic);
  }

  // execute the new init() function of the resource
  if(resource->init){
     resource->init();
  }
}
/*---------------------------------------------------------------------------*/
/*- Internal API ------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
list_t
rest_get_resources(void)
{
  return restful_services;
}
list_t
rest_get_periodic_resources(void)
{
  return restful_periodic_services;
}
/*---------------------------------------------------------------------------*/
int
rest_invoke_restful_service(void *request, void *response, uint8_t *buffer,
                            uint16_t buffer_size, int32_t *offset)
{
  uint8_t found = 0;
  uint8_t allowed = 1;

  resource_t *resource = NULL;
  const char *url = NULL;
  int url_len, res_url_len;

  url_len = REST.get_url(request, &url);
  for(resource = (resource_t *)list_head(restful_services);
      resource; resource = resource->next) {

    /* if the web service handles that kind of requests and urls matches */
    res_url_len = strlen(resource->url);
    if((url_len == res_url_len
        || (url_len > res_url_len
            && (resource->flags & HAS_SUB_RESOURCES)
            && url[res_url_len] == '/'))
       && strncmp(resource->url, url, res_url_len) == 0) {
      found = 1;
      rest_resource_flags_t method = REST.get_method_type(request);

      PRINTF("/%s, method %u, resource->flags %u\n", resource->url,
             (uint16_t)method, resource->flags);

      if((method & METHOD_GET) && resource->get_handler != NULL) {
        /* call handler function */
        resource->get_handler(request, response, buffer, buffer_size, offset);
      } else if((method & METHOD_POST) && resource->post_handler != NULL) {
        /* call handler function */
        resource->post_handler(request, response, buffer, buffer_size,
                               offset);
      } else if((method & METHOD_PUT) && resource->put_handler != NULL) {
        /* call handler function */
        resource->put_handler(request, response, buffer, buffer_size, offset);
      } else if((method & METHOD_DELETE) && resource->delete_handler != NULL) {
        /* call handler function */
        resource->delete_handler(request, response, buffer, buffer_size,
                                 offset);
      } else {
        allowed = 0;
        REST.set_response_status(response, REST.status.METHOD_NOT_ALLOWED);
      }
      break;
    }
  }
  if(!found) {
    REST.set_response_status(response, REST.status.NOT_FOUND);
  } else if(allowed) {
    /* final handler for special flags */
    if(resource->flags & IS_OBSERVABLE) {
      REST.subscription_handler(resource, request, response);
    }
  }
  return found & allowed;
}
/*-----------------------------------------------------------------------------------*/
#if REST_DELAY_RES_START
static void
observer_periodic(void)
{
  periodic_resource_t *periodic_resource = NULL;
  coap_observer_t *obs = NULL;
  uint8_t obsfound;
  uint8_t nb_observers = 0;

  for(periodic_resource = list_head(rest_get_periodic_resources());
      periodic_resource; periodic_resource = periodic_resource->next)
  {
    PRINTF("REST: (observer periodic) Available periodic resource: '%s' with period %lu.\n",
        periodic_resource->resource->url, periodic_resource->period);
    if( periodic_resource->period )
    {
      /* search within the observers if the resource is still observed
       * otherwise disable the periodic timer of the resource. */
      obsfound = 0;
      for(obs = coap_get_list_observers(); obs; obs = obs->next)
      {
        nb_observers++;
        if( !strcmp(obs->url, periodic_resource->resource->url )  )
        {
          obsfound = 1;
          break;
        }
      }

      if( !obsfound )
      {
        // Stop the periodic resource
        PRINTF("REST: (observer periodic) Observer not found for periodic resource '%s' -> Stop timer.\n",
            periodic_resource->resource->url);
        etimer_stop(&periodic_resource->periodic_timer);
      }
    }

  } // end for

  /* Don't need to keep the observer periodic if there is zero observer. */
  if( !nb_observers ){
    etimer_stop(&observer_periodic_timer);
  } else {
    etimer_reset(&observer_periodic_timer);
  }
}
/*-----------------------------------------------------------------------------------*/
static void
start_observer_periodic(void)
{
  /* We call the function the first time only to store the process
   * where the timer event is handled. The observer periodic is effectively
   * started when a first periodic resource interval is set. */
  if( observer_periodic_timer_p == NULL ){
    // store the process which called the function the first time
    observer_periodic_timer_p = PROCESS_CURRENT();
    return;
  }

  if( !etimer_expired(&observer_periodic_timer) )
  {
    /* We restart the observer periodic timer to prevent the auto-removal by this latter
     * of the resource timer if an observer is not immediately registered.
     * An observer has from now OBSERVER_PERIODIC seconds to subscribe to the resource. */
    PROCESS_CONTEXT_BEGIN(observer_periodic_timer_p);
    etimer_restart(&observer_periodic_timer);
    PROCESS_CONTEXT_END(observer_periodic_timer_p);
  } else {
    PROCESS_CONTEXT_BEGIN(observer_periodic_timer_p);
    etimer_set(&observer_periodic_timer, CLOCK_SECOND * REST_OBSERVER_PERIODIC);
    PROCESS_CONTEXT_END(observer_periodic_timer_p);
    PRINTF("RES: Observer periodic started.\n");
  }
}
void
rest_update_resource_interval(resource_t *resource, uint32_t interval)
{
#if REST_RESOURCES_DESYNCH
  periodic_resource_t *periodic_resource = NULL;
  periodic_resource_t *periodic_resource_temp = NULL;
  clock_time_t tdiff;
  uint32_t intervals[MAX_PERIODIC_RESSOURCES];
  periodic_resource_t *periodic_resources[MAX_PERIODIC_RESSOURCES];
  uint32_t best_interval = 0;
  uint8_t active_periodic_resources;
  uint8_t i, j;
#endif /* REST_RESOURCES_DESYNCH */

  resource->periodic->period =  interval * CLOCK_SECOND;

  if( interval )
  {
    /* The periodic resources are independant from observer. They can fire without a
     * specific observer registered to the resource. Therefore, we enable here a periodic
     * check of resource vs observer in order to disable the periodic resource timer if no
     * observer is found for this periodic resource for a while. */
    start_observer_periodic();

    PROCESS_CONTEXT_BEGIN(&rest_engine_process);
    etimer_set(&resource->periodic->periodic_timer, resource->periodic->period);
    PROCESS_CONTEXT_END(&rest_engine_process);
  } else {
    etimer_stop(&resource->periodic->periodic_timer);
  } /* if( interval ) */

#if REST_RESOURCES_DESYNCH

  active_periodic_resources = 0;
  for(periodic_resource = get_restful_periodic_services();
    periodic_resource; periodic_resource = periodic_resource->next)
  {
    if( periodic_resource->period )
    {
      intervals[active_periodic_resources] = periodic_resource->period;
      periodic_resources[active_periodic_resources++] = periodic_resource;
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
        if (timer_remaining(&(periodic_resources[i]->periodic_timer.timer)) >
          timer_remaining(&(periodic_resources[j]->periodic_timer.timer)))
        {
          periodic_resource_temp =  periodic_resources[i];
          periodic_resources[i] = periodic_resources[j];
          periodic_resources[j] = periodic_resource_temp;
        }
      }
    }

    /* Adjust resources periodic wake-up */
    for (i = 0; i < active_periodic_resources-1; ++i)
    {

      tdiff = timer_remaining(&(periodic_resources[i+1]->periodic_timer.timer)) - \
          timer_remaining(&(periodic_resources[i]->periodic_timer.timer));

      if((int32_t)tdiff < 0){
        tdiff = 0;
      }

      etimer_adjust(&periodic_resources[i+1]->periodic_timer, best_interval-tdiff);

    } /* for() */

  } /* if() */

#if DEBUG
  PRINTF("RES: Best interval between resources is %lu (%s seconds).\n", best_interval,
      float2str((float)best_interval/CLOCK_SECOND, 1));
  for (i = 0; i < active_periodic_resources; ++i)
  {
    PRINTF("RES: Resource '%s' wakes-up in %s seconds.\n", periodic_resources[i]->resource->url,
        float2str((float)timer_remaining(&(periodic_resources[i]->periodic_timer.timer))/CLOCK_SECOND, 1));
  }
#endif

#endif /* REST_RESOURCES_DESYNCH */

}
#endif /* REST_DELAY_RES_START */
/*-----------------------------------------------------------------------------------*/
PROCESS_THREAD(rest_engine_process, ev, data)
{
  PROCESS_BEGIN();

#if REST_DELAY_RES_START
  start_observer_periodic();
#endif /* REST_DELAY_RES_START */

  /* pause to let REST server finish adding resources. */
  PROCESS_PAUSE();

  /* initialize the PERIODIC_RESOURCE timers, which will be handled by this process. */
  periodic_resource_t *periodic_resource = NULL;

#if !REST_DELAY_RES_START
  for(periodic_resource =
        (periodic_resource_t *)list_head(restful_periodic_services);
      periodic_resource; periodic_resource = periodic_resource->next) {
    if(periodic_resource->periodic_handler && periodic_resource->period) {
      PRINTF("Periodic: Set timer for /%s to %lu\n",
             periodic_resource->resource->url, periodic_resource->period);
      etimer_set(&periodic_resource->periodic_timer,
                 periodic_resource->period);
    }
  }
#endif /* !REST_DELAY_RES_START */
  while(1) {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER) {

#if REST_DELAY_RES_START
      if( data == &observer_periodic_timer &&
          etimer_expired(&observer_periodic_timer) )
      {
          observer_periodic();
          continue;
      }
#endif /* REST_DELAY_RES_START */

      for(periodic_resource =
            (periodic_resource_t *)list_head(restful_periodic_services);
          periodic_resource; periodic_resource = periodic_resource->next) {
        if(periodic_resource->period
           && etimer_expired(&periodic_resource->periodic_timer)) {

          PRINTF("Periodic: etimer expired for /%s (period: %lu)\n",
                 periodic_resource->resource->url, periodic_resource->period);

          /* Call the periodic_handler function, which was checked during adding to list. */
          (periodic_resource->periodic_handler)();

          etimer_reset(&periodic_resource->periodic_timer);
        }
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
