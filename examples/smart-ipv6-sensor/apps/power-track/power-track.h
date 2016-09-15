/**
 * \addtogroup apps
 * @{
 *
 * \defgroup power-track Power tracking apps
 * @{
 *
 * \file
 *  Header file for the power tracking application
 *
 * \author
 * Darko Petrovic
 *
 */
#ifndef _POWER_TRACK_H
#define _POWER_TRACK_H

typedef struct energest_data {

  uint32_t all_cpu;
  uint32_t all_lpm;
  uint32_t all_transmit;
  uint32_t all_listen;
  uint32_t all_flash_read;
  uint32_t all_flash_write;
  uint32_t all_flash_erase;
  uint32_t all_sensors_ina3221;
  uint32_t all_sensors_sht21;
  uint32_t all_sensors_tmp100;
  uint32_t all_sensors_pir;
#if CONTIKIMAC_CONF_COMPOWER
  uint32_t idle_transmit;
  uint32_t idle_listen;
  uint32_t all_idle_transmit;
  uint32_t all_idle_listen;
#endif
  uint32_t all_led_red;
  uint32_t all_led_yellow;
  uint32_t all_time;
  uint32_t all_leds;
  uint32_t cpu;
  uint32_t lpm;
  uint32_t transmit;
  uint32_t listen;
  uint32_t avg_current;       // in Amper
  float charge_consumed;      // in Coulomb
#ifdef PLATFORM_HAS_BATTERY
  float remaining_charge;     // in Coulomb
  uint32_t estimated_lifetime;
#endif
} energest_data_t;

extern energest_data_t energest_data;

/**
 * \name Current consumption in A
 *
 * @{
 */
#define I_CPU     0.010     // measured
#define I_LPM     0.000005    // measured
#define I_TX      0.023     // measured (-24dBm)
#define I_RX      0.019     // measured
#define I_INA3221 0.000326    // measured
#define I_SHT21   0.000277    // measured
#define I_TMP100  0.000047    // measured
#define I_PIR     0.0000038   // measured
#define I_LED     0.000776    // measured

/** @} */


void powertrack_start(clock_time_t period);
void powertrack_stop(void);
void powertrack_reset(void);

#endif /* _POWER_TRACK_H */

/** @} */
/** @} */
