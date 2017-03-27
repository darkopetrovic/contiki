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

#if CONTIKI_TARGET_SSIPV6S_V1 || CONTIKI_TARGET_SSIPV6S_V2
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
#if CONTIKI_TARGET_SSIPV6S_V2
static uint32_t last_ccs811, last_pir, last_mic;
#endif
#if CONTIKIMAC_CONF_COMPOWER
static uint32_t last_idle_transmit, last_idle_listen;
#endif

static void (* callback)(void);

static void
energest_compute(void)
{
#if PLATFORM_HAS_BATTERY && (CONTIKI_TARGET_SSIPV6S_V1 || CONTIKI_TARGET_SSIPV6S_V2)
  uint16_t battery_volt_mv;
  float battery_volt;
  battery_volt_mv = get_battery_voltage();
  battery_volt = (float)battery_volt_mv/1000;
#endif

  uint32_t all_sensors_with_lpm = 0;
  uint32_t all_lpm;

  PRINTF("Compute energest.\n");

  energest_flush();

  /******************* ALL values calculation ****************************/

  energest_data.all_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  all_lpm = energest_type_time(ENERGEST_TYPE_LPM);
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

  energest_data.all_time = energest_data.all_cpu + energest_data.all_lpm;
  energest_data.all_leds = energest_data.all_led_red + energest_data.all_led_yellow;

  /* Add here the new types */

#if CONTIKI_TARGET_SSIPV6S_V1
  energest_data.all_sensors_ina3221 = energest_type_time(ENERGEST_TYPE_SENSORS_INA3221);
  energest_data.all_sensors_sht21 = energest_type_time(ENERGEST_TYPE_SENSORS_SHT21);
  energest_data.all_sensors_tmp100 = energest_type_time(ENERGEST_TYPE_SENSORS_TMP100);
  energest_data.all_sensors_pir = energest_type_time(ENERGEST_TYPE_SENSORS_PIR);

  all_sensors_with_lpm = energest_data.all_sensors_ina3221 + energest_data.all_sensors_sht21 +
        energest_data.all_sensors_tmp100;

#endif

#if CONTIKI_TARGET_SSIPV6S_V2
  energest_data.all_sensors_ina3221 = energest_type_time(ENERGEST_TYPE_SENSORS_INA3221);
  energest_data.all_sensors_sht21 = energest_type_time(ENERGEST_TYPE_SENSORS_SHT21);
  energest_data.all_sensors_bmp280 = energest_type_time(ENERGEST_TYPE_SENSORS_BMP280);
  energest_data.all_sensors_tsl2561 = energest_type_time(ENERGEST_TYPE_SENSORS_TSL2561);

  /* When performing sensor measurement, the SoC is put in LPM2 mode,
    * therefore the consumption of the sensors contains the LPM consumption.
    * We need to substract all sensors from the lpm to consider only the LPM alone.
    */
  all_sensors_with_lpm = energest_data.all_sensors_ina3221 + energest_data.all_sensors_sht21
       + energest_data.all_sensors_bmp280 + energest_data.all_sensors_tsl2561;

  // special treatment because they are constantly power on when activated
  // and the variable overlaps after 131072 seconds (2^32 / RTIMER_SECOND)
  energest_data.all_sensors_ccs811 = energest_type_time(ENERGEST_TYPE_SENSORS_CCS811);

  // variable has overlaped

  if(last_ccs811 > energest_type_time(ENERGEST_TYPE_SENSORS_CCS811)){
    energest_data.all_sensors_mic += ((0xFFFFFFFF-last_ccs811)+energest_type_time(ENERGEST_TYPE_SENSORS_CCS811))/RTIMER_SECOND;
  } else {
    energest_data.all_sensors_mic += (energest_type_time(ENERGEST_TYPE_SENSORS_CCS811)-last_ccs811)/RTIMER_SECOND;
  }

  if(last_pir > energest_type_time(ENERGEST_TYPE_SENSORS_PIR)){
    energest_data.all_sensors_pir += ((0xFFFFFFFF-last_pir)+energest_type_time(ENERGEST_TYPE_SENSORS_PIR))/RTIMER_SECOND;
  } else {
    energest_data.all_sensors_pir += (energest_type_time(ENERGEST_TYPE_SENSORS_PIR)-last_pir)/RTIMER_SECOND;
  }

  if(last_mic > energest_type_time(ENERGEST_TYPE_SENSORS_MIC)){
    energest_data.all_sensors_mic += ((0xFFFFFFFF-last_mic)+energest_type_time(ENERGEST_TYPE_SENSORS_MIC))/RTIMER_SECOND;
  } else {
    energest_data.all_sensors_mic += (energest_type_time(ENERGEST_TYPE_SENSORS_MIC)-last_mic)/RTIMER_SECOND;
  }

#endif

  /* Normally sensors measurement duration shouldn't be bigger than the LPM duration
   * since the LPM is activated during measurement, but since in ROUTER mode the LPM
   * isn't activated and measurement are performed, we must not substract from a 0 value.
   */
  if( all_lpm > (energest_data.all_leds+all_sensors_with_lpm)){
    all_lpm -= (energest_data.all_leds+all_sensors_with_lpm);
  }

  if(last_lpm > all_lpm){
    energest_data.all_lpm += ((0xFFFFFFFF-last_lpm)+all_lpm)/RTIMER_SECOND;
  } else {
    energest_data.all_lpm += (all_lpm-last_lpm)/RTIMER_SECOND;
  }

  /**************** CURRENT values calculation **************************/

  energest_data.cpu = energest_data.all_cpu - last_cpu;
  energest_data.lpm = energest_type_time(ENERGEST_TYPE_LPM) - last_lpm;
  energest_data.transmit = energest_data.all_transmit - last_transmit;
  energest_data.listen = energest_data.all_listen - last_listen;
#if CONTIKIMAC_CONF_COMPOWER
  energest_data.idle_transmit = compower_idle_activity.transmit - last_idle_transmit;
  energest_data.idle_listen = compower_idle_activity.listen - last_idle_listen;
#endif

  /**************** LAST values calculation *************************/

  last_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  last_lpm = all_lpm;
  last_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  last_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
#if CONTIKIMAC_CONF_COMPOWER
  last_idle_listen = compower_idle_activity.listen;
  last_idle_transmit = compower_idle_activity.transmit;
#endif

#if CONTIKI_TARGET_SSIPV6S_V2
  last_ccs811 = energest_type_time(ENERGEST_TYPE_SENSORS_CCS811);
  last_pir = energest_type_time(ENERGEST_TYPE_SENSORS_PIR);
  last_mic = energest_type_time(ENERGEST_TYPE_SENSORS_MIC);
#endif

  /*
   * LPM and CPU states covers all other states.
   * The CPU is put in LPM when doing SENSORS measurments and LED blinking.
   * TX and RX currents takes in account the CPU current.
   * */

  energest_data.charge_consumed =
      (float)(energest_data.all_cpu-energest_data.all_transmit-energest_data.all_listen)/RTIMER_SECOND * I_CPU \
      + (float)energest_data.all_lpm * I_LPM \
      + (float)energest_data.all_transmit/RTIMER_SECOND * I_TX \
      + (float)energest_data.all_listen/RTIMER_SECOND * I_RX
#if CONTIKI_TARGET_SSIPV6S_V1
      + (float)energest_data.all_sensors_ina3221/RTIMER_SECOND * I_INA3221 \
      + (float)energest_data.all_sensors_sht21/RTIMER_SECOND * I_SHT21 \
      + (float)energest_data.all_sensors_tmp100/RTIMER_SECOND * I_TMP100 \
      + (float)energest_data.all_sensors_pir/RTIMER_SECOND * I_PIR
#endif
#if CONTIKI_TARGET_SSIPV6S_V2
      + (float)energest_data.all_sensors_ina3221/RTIMER_SECOND * I_INA3221 \
      + (float)energest_data.all_sensors_sht21/RTIMER_SECOND * I_SHT21 \
      + (float)energest_data.all_sensors_bmp280/RTIMER_SECOND * I_BMP280 \
      + (float)energest_data.all_sensors_tsl2561/RTIMER_SECOND * I_TSL2561 \
      + (float)energest_data.all_sensors_ccs811 * I_CCS811 \
      + (float)energest_data.all_sensors_pir * I_PIR \
      + (float)energest_data.all_sensors_mic * I_MIC
#endif
      + (float)energest_data.all_leds/RTIMER_SECOND * I_LED;

#if PLATFORM_HAS_BATTERY && (CONTIKI_TARGET_SSIPV6S_V1 || CONTIKI_TARGET_SSIPV6S_V2)
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
  etimer_set(&periodic, *period*CLOCK_SECOND);

  while(1) {
    PROCESS_WAIT_UNTIL(etimer_expired(&periodic));
    etimer_reset(&periodic);

    energest_compute();

    energest_data.avg_current = \
      (energest_data.charge_consumed - charge_consumed_xago)/(*period)*1e6;
#ifdef PLATFORM_HAS_BATTERY
    energest_data.estimated_lifetime = \
        energest_data.remaining_charge*1e6 / energest_data.avg_current;
#endif
    charge_consumed_xago = energest_data.charge_consumed;

    callback();
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
powertrack_update_period(clock_time_t period)
{
  if(process_is_running(&powertrack_process)){
    PROCESS_CONTEXT_BEGIN(&powertrack_process);
    etimer_set(&periodic, period*CLOCK_SECOND);
    PROCESS_CONTEXT_END(&powertrack_process);
  }
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

void
powertrack_set_callback(void* cb)
{
  callback = cb;
}

#endif /* ENERGEST_CONF_ON */
