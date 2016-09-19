/**
 * \addtogroup power-track
 * @{
 *
 * \file
 *
 *
 *  \author
 *  Darko Petrovic
 */

#include "contiki.h"
#include "power-track.h"

#if CONTIKI_TARGET_SSIPV6S_V1
#include "platform-sensors.h"
#endif

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if ENERGEST_CONF_ON

#if CONTIKIMAC_CONF_COMPOWER
#include "sys/compower.h"
#endif

PROCESS(powertrack_process, "Periodic power tracking");

static struct etimer periodic;
energest_data_t energest_data;
static double charge_consumed_xago;
static uint32_t last_cpu, last_lpm, last_transmit, last_listen;
#if CONTIKIMAC_CONF_COMPOWER
static uint32_t last_idle_transmit, last_idle_listen;
#endif

static void
energest_compute(void)
{
#ifdef PLATFORM_HAS_BATTERY
  uint16_t battery_volt_mv;
  float battery_volt;
  battery_volt_mv = get_battery_voltage();
  battery_volt = (float)battery_volt_mv/1000;
#endif

  uint32_t sup_lpm = 0;

  energest_flush();

  /* ALL values calculation */

  energest_data.all_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  energest_data.all_lpm = energest_type_time(ENERGEST_TYPE_LPM);
  energest_data.all_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  energest_data.all_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
#if CONTIKIMAC_CONF_COMPOWER
  energest_data.all_idle_transmit = compower_idle_activity.transmit;
  energest_data.all_idle_listen = compower_idle_activity.listen;
#endif

  energest_data.all_led_red = energest_type_time(ENERGEST_TYPE_LED_RED);
  energest_data.all_led_yellow = energest_type_time(ENERGEST_TYPE_LED_YELLOW);
  energest_data.all_flash_read = energest_type_time(ENERGEST_TYPE_FLASH_READ);
  energest_data.all_flash_write = energest_type_time(ENERGEST_TYPE_FLASH_WRITE);
  energest_data.all_flash_erase = energest_type_time(ENERGEST_TYPE_FLASH_ERASE);

  /* Add here the new types */

#if CONTIKI_TARGET_SSIPV6S_V1
  energest_data.all_sensors_ina3221 = energest_type_time(ENERGEST_TYPE_SENSORS_INA3221);
  energest_data.all_sensors_sht21 = energest_type_time(ENERGEST_TYPE_SENSORS_SHT21);
  energest_data.all_sensors_tmp100 = energest_type_time(ENERGEST_TYPE_SENSORS_TMP100);
  energest_data.all_sensors_pir = energest_type_time(ENERGEST_TYPE_SENSORS_PIR);
#endif


  /* CURRENT values calculation */

  energest_data.cpu = energest_data.all_cpu - last_cpu;
  energest_data.lpm = energest_data.all_lpm - last_lpm;
  energest_data.transmit = energest_data.all_transmit - last_transmit;
  energest_data.listen = energest_data.all_listen - last_listen;
#if CONTIKIMAC_CONF_COMPOWER
  energest_data.idle_transmit = compower_idle_activity.transmit - last_idle_transmit;
  energest_data.idle_listen = compower_idle_activity.listen - last_idle_listen;
#endif

  /* LAST values calculation */

  last_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  last_lpm = energest_type_time(ENERGEST_TYPE_LPM);
  last_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  last_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
#if CONTIKIMAC_CONF_COMPOWER
  last_idle_listen = compower_idle_activity.listen;
  last_idle_transmit = compower_idle_activity.transmit;
#endif

  //time = energest_data.cpu + energest_data.lpm;
  energest_data.all_time = energest_data.all_cpu + energest_data.all_lpm;
  energest_data.all_leds = energest_data.all_led_red + energest_data.all_led_yellow;

  /*
   * LPM and CPU states covers all other states.
   * The CPU is put in LPM when doing SENSORS measurments and LED blinking.
   * TX and RX currents takes in account the CPU current.
   * */

  /* myBUG: sometimes, especially during the first minute of the device,
   * RX+TX duration is greater than CPU which shouldn't be possible. */

#if CONTIKI_TARGET_SSIPV6S_V1
  sup_lpm = energest_data.all_sensors_ina3221 + energest_data.all_sensors_sht21 +
      energest_data.all_sensors_tmp100;
#endif

  energest_data.charge_consumed =
      (float)(energest_data.all_cpu-energest_data.all_transmit-energest_data.all_listen)/RTIMER_SECOND * I_CPU \
      + (float)(energest_data.all_lpm-energest_data.all_leds-sup_lpm)/RTIMER_SECOND * I_LPM \
      + (float)energest_data.all_transmit/RTIMER_SECOND * I_TX \
      + (float)energest_data.all_listen/RTIMER_SECOND * I_RX
#if CONTIKI_TARGET_SSIPV6S_V1
      + (float)energest_data.all_sensors_ina3221/RTIMER_SECOND * I_INA3221 \
      + (float)energest_data.all_sensors_sht21/RTIMER_SECOND * I_SHT21 \
      + (float)energest_data.all_sensors_tmp100/RTIMER_SECOND * I_TMP100 \
      + (float)energest_data.all_sensors_pir/RTIMER_SECOND * I_PIR
#endif
      + (float)energest_data.all_leds/RTIMER_SECOND * I_LED;


#ifdef PLATFORM_HAS_BATTERY
  /* Estimate the remaining battery capacity using simple relationship with the battery level:
   *  BATTERY_NOM_VOLTAGE -> BATTERY_CAPACITY mAh ->  x 3600 = Coulomb
    BATTERY_CUT_VOLTAGE -> 0 mAh ->  0 Coulomb */
  energest_data.remaining_charge = (battery_volt*BATTERY_CAPACITY/(BATTERY_NOM_VOLTAGE-BATTERY_CUT_VOLTAGE)-\
      (BATTERY_NOM_VOLTAGE*BATTERY_CAPACITY/(BATTERY_NOM_VOLTAGE-BATTERY_CUT_VOLTAGE)-BATTERY_CAPACITY)) \
      * 3600;
#endif

}
PROCESS_THREAD(powertrack_process, ev, data)
{
  clock_time_t *period = NULL;

  PROCESS_BEGIN();

  period = data;

  if(period == NULL) {
    PROCESS_EXIT();
  }
  etimer_set(&periodic, *period);

  while(1) {
    PROCESS_WAIT_UNTIL(etimer_expired(&periodic));
    etimer_reset(&periodic);

    energest_compute();

    energest_data.avg_current = \
      (energest_data.charge_consumed - charge_consumed_xago)/(*period/CLOCK_SECOND)*1e6;
#ifdef PLATFORM_HAS_BATTERY
    energest_data.estimated_lifetime = \
        energest_data.remaining_charge*1e6 / energest_data.avg_current;
#endif
    charge_consumed_xago = energest_data.charge_consumed;
  }

  PROCESS_END();
}
void
powertrack_start(clock_time_t period)
{
  PRINTF("Start power tracking with period %lu.\n", period);
  process_start(&powertrack_process, (void *)&period);
}
/*---------------------------------------------------------------------------*/
void
powertrack_stop(void)
{
  PRINTF("Stopping power tracking.\n");
  process_exit(&powertrack_process);
}

void
powertrack_reset(void)
{
  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);
#if CONTIKIMAC_CONF_COMPOWER
  compower_init();
#endif

  /* Set all values to 0. */
  memset((void*)&energest_data, 0, sizeof(energest_data_t));
  last_cpu = 0;
  last_lpm = 0;
  last_transmit = 0;
  last_listen = 0;
#if CONTIKIMAC_CONF_COMPOWER
  last_idle_listen = 0;
  last_idle_transmit = 0;
#endif
  charge_consumed_xago = 0;

  PROCESS_CONTEXT_BEGIN(&powertrack_process);
  etimer_restart(&periodic);
  PROCESS_CONTEXT_END(&powertrack_process);
}

#endif /* ENERGEST_CONF_ON */
