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
 * \addtogroup ipso-objects
 * @{
 */

/**
 * \file
 *         Implementation of OMA LWM2M / IPSO Humidity
 * \author
 *         Darko Petrovic <darko.petrovic@hevs.ch>
 *
 */

#include <stdint.h>
#include "ipso-objects.h"
#include "lwm2m-object.h"
#include "lwm2m-engine.h"
#include "er-coap-engine.h"

#include "ina3221-sensor.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define IPSO_VOLTAGE_MIN (0 * LWM2M_FLOAT32_FRAC)
#define IPSO_VOLTAGE_MAX (4 * LWM2M_FLOAT32_FRAC)

static struct ctimer periodic_timer_battery;
static struct ctimer periodic_timer_solar;
static int32_t min_battery_value;
static int32_t max_battery_value;
static int32_t min_solar_value;
static int32_t max_solar_value;
static int32_t interval_battery=10;
static int32_t interval_solar=10;
static int read_battery_voltage(int32_t *value);
static int read_solar_voltage(int32_t *value);
static void handle_periodic_timer_battery(void *ptr);
static void handle_periodic_timer_solar(void *ptr);

/*---------------------------------------------------------------------------*/
static int
battery_value(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  if(read_battery_voltage(&value)) {
    return ctx->writer->write_float32fix(ctx, outbuf, outsize,
                                         value, LWM2M_FLOAT32_BITS);
  }
  return 0;
}

static int
solar_value(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  int32_t value;
  if(read_solar_voltage(&value)) {
    return ctx->writer->write_float32fix(ctx, outbuf, outsize,
                                         value, LWM2M_FLOAT32_BITS);
  }
  return 0;
}

static int
read_sampling1(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, interval_battery);
}

static int
write_sampling1(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
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
    interval_battery = value;
  }

  if(value && periodic_timer_battery.etimer.p != PROCESS_NONE){
    ctimer_set(&periodic_timer_battery, CLOCK_SECOND * interval_battery, handle_periodic_timer_battery, NULL);
  } else {
    ctimer_stop(&periodic_timer_battery);
  }

  return len;
}

static int
exec_sampling1(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  ctimer_set(&periodic_timer_battery, CLOCK_SECOND * interval_battery, handle_periodic_timer_battery, NULL);
  return 1;
}

static int
read_sampling2(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
  return ctx->writer->write_int(ctx, outbuf, outsize, interval_solar);
}

static int
write_sampling2(lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
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
    interval_battery = value;
  }

  if(value && periodic_timer_solar.etimer.p != PROCESS_NONE){
    ctimer_set(&periodic_timer_solar, CLOCK_SECOND * interval_solar, handle_periodic_timer_solar, NULL);
  } else {
    ctimer_stop(&periodic_timer_solar);
  }

  return len;
}

static int
exec_sampling2(lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
               uint8_t *outbuf, size_t outlen)
{
  ctimer_set(&periodic_timer_solar, CLOCK_SECOND * interval_solar, handle_periodic_timer_solar, NULL);
  return 1;
}

/*---------------------------------------------------------------------------*/
LWM2M_RESOURCES(voltage_resources_battery,
                /* Temperature (Current) */
                LWM2M_RESOURCE_CALLBACK(5700, { battery_value, NULL, NULL }),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "V"),
                LWM2M_RESOURCE_STRING(5750, "Battery voltage"),
                /* Min Range Value */
                LWM2M_RESOURCE_FLOATFIX(5603, IPSO_VOLTAGE_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_FLOATFIX(5604, IPSO_VOLTAGE_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5601, &min_battery_value),
                /* Max Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5602, &max_battery_value),

                LWM2M_RESOURCE_CALLBACK(REURES_SAMPLING_INTERVAL, { read_sampling1, write_sampling1, exec_sampling1 })
                );

LWM2M_RESOURCES(voltage_resources_solar,
                /* Temperature (Current) */
                LWM2M_RESOURCE_CALLBACK(5700, { solar_value, NULL, NULL }),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "V"),
                LWM2M_RESOURCE_STRING(5750, "Solar panel voltage"),
                /* Min Range Value */
                LWM2M_RESOURCE_FLOATFIX(5603, IPSO_VOLTAGE_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_FLOATFIX(5604, IPSO_VOLTAGE_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5601, &min_solar_value),
                /* Max Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5602, &max_solar_value),

                LWM2M_RESOURCE_CALLBACK(REURES_SAMPLING_INTERVAL, { read_sampling2, write_sampling2, exec_sampling2 })
                );


LWM2M_INSTANCES(voltage_instances,
                LWM2M_INSTANCE(0, voltage_resources_battery),
                LWM2M_INSTANCE(1, voltage_resources_solar));
LWM2M_OBJECT(voltage, 3316, voltage_instances);
/*---------------------------------------------------------------------------*/
static int
read_battery_voltage(int32_t *value)
{
  uint16_t sensors_value;

#if 0
  if(USB_IS_PLUGGED()){

    /**
     * myTODO: Couln't disable pin interrupt here with any of the following:
     * - INTERRUPTS_DISABLE();
     * - nvic_interrupt_unpend(USB_PLUG_DETECT_VECTOR);
     * - nvic_interrupt_disable(USB_PLUG_DETECT_VECTOR);
     * - GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(USB_PLUG_DETECT_PORT), GPIO_PIN_MASK(USB_PLUG_DETECT_PIN));
     * - GPIO_DISABLE_POWER_UP_INTERRUPT(GPIO_PORT_TO_BASE(USB_PLUG_DETECT_PORT), GPIO_PIN_MASK(USB_PLUG_DETECT_PIN));
     *
     * */

    USB_REG_DISABLE();
    reading_voltage = 1;
    deep_sleep_ms(300, NO_GPIO_INTERRUPT, 0);
  } else {
    deep_sleep_ms(125, NO_GPIO_INTERRUPT, 0);
  }
#endif

  // read sensor value here
  SENSORS_ACTIVATE(ina3221_sensor);
  SENSORS_MEASURE(ina3221_sensor);
  sensors_value = ina3221_sensor.value(INA3221_CH2_BUS_VOLTAGE);
  SENSORS_DEACTIVATE(ina3221_sensor);

  /* Convert to fix float */
  *value = (sensors_value * LWM2M_FLOAT32_FRAC) / 1000;

  if(*value < min_battery_value) {
    min_battery_value = *value;
    lwm2m_object_notify_observers(&voltage, "/0/5601");
  }
  if(*value > max_battery_value) {
    max_battery_value = *value;
    lwm2m_object_notify_observers(&voltage, "/0/5602");
  }
  return 1;

}

static int
read_solar_voltage(int32_t *value)
{
  uint16_t sensors_value;

#if 0
  if(USB_IS_PLUGGED()){
    USB_REG_DISABLE();
    reading_voltage = 1;
    deep_sleep_ms(300, NO_GPIO_INTERRUPT, 0);
  }
#endif

  // read sensor value here
  SENSORS_ACTIVATE(ina3221_sensor);
  SENSORS_MEASURE(ina3221_sensor);
  sensors_value = ina3221_sensor.value(INA3221_CH1_BUS_VOLTAGE);
  SENSORS_DEACTIVATE(ina3221_sensor);

  /* Convert to fix float */
  *value = (sensors_value * LWM2M_FLOAT32_FRAC) / 1000;

  if(*value < min_solar_value) {
    min_solar_value = *value;
    lwm2m_object_notify_observers(&voltage, "/1/5601");
  }
  if(*value > max_solar_value) {
    max_solar_value = *value;
    lwm2m_object_notify_observers(&voltage, "/1/5602");
  }
  return 1;

}
/*---------------------------------------------------------------------------*/
static void
handle_periodic_timer_battery(void *ptr)
{
  lwm2m_object_notify_observers(&voltage, "/0/5700");
  ctimer_reset(&periodic_timer_battery);
}

static void
handle_periodic_timer_solar(void *ptr)
{
  lwm2m_object_notify_observers(&voltage, "/1/5700");
  ctimer_reset(&periodic_timer_solar);
}
/*---------------------------------------------------------------------------*/
void
ipso_voltage_init(void)
{
  min_battery_value = IPSO_VOLTAGE_MAX;
  max_battery_value = IPSO_VOLTAGE_MIN;
  min_solar_value = IPSO_VOLTAGE_MAX;
  max_solar_value = IPSO_VOLTAGE_MIN;

  /* register this device and its handlers - the handlers automatically
     sends in the object to handle */
  lwm2m_engine_register_object(&voltage);
}
/*---------------------------------------------------------------------------*/
/** @} */
