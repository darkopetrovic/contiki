/**
 * \addtogroup sensor-ccs811
 * @{
 *
 * \file
 * Driver for the CCS811 sensor
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 */

#include "ccs811-sensor.h"
#include "pca9546-mux.h"

/** \cond */
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/** \endcond */

static uint16_t locBaseline = 0x85b2;   // based on test on first sample
static uint8_t  locTemperature = 0x64;      // default to 25 degree
static uint8_t  locHumidity = 0x64;     // default to 50%
static uint8_t  locModeOfMeasure = CCS811_MODE_10_SECONDS;  // measure each second
static uint16_t co2;
static uint16_t tvoc;

/*!**********************************************************************************
 * \brief       CCS I2C enable nWAKE
 *
 ***********************************************************************************/
static void
CCS_I2C_enable(void)
{
  GPIO_SET_OUTPUT( CCS811_NWAKE_PORT_BASE, CCS811_NWAKE_PIN_MASK);
  GPIO_CLR_PIN( CCS811_NWAKE_PORT_BASE, CCS811_NWAKE_PIN_MASK); // activate
  clock_delay_usec(50);   // datasheet delay before I2C access
}

/*!**********************************************************************************
 * \brief       CCS I2C disable nWAKE
 *
 ***********************************************************************************/
static void
CCS_I2C_disable(void)
{
  clock_delay_usec(35);   // nothing
  GPIO_SET_OUTPUT( CCS811_NWAKE_PORT_BASE, CCS811_NWAKE_PIN_MASK);
  GPIO_SET_PIN( CCS811_NWAKE_PORT_BASE, CCS811_NWAKE_PIN_MASK); // activate
  clock_delay_usec(35);   // datasheet delay minimum deasserted
}

/*!**********************************************************************************
 * \brief       Read sensor values
 *
 * \param reg     Location where to store the values
 *
 * \return      \ref CCS811_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_sensor(uint8_t *reg)
{
  uint16_t err;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_enable();    // activate I2C

  if((err = i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_ALG_RESULT_DATA)) != I2C_MASTER_ERR_NONE){
    CCS_I2C_disable();    // desactivate I2C
    return err | CCS811_ERR_WRITE_REGISTER<<8;
  }
  if((err = i2c_burst_receive(CCS811_SLAVE_ADDRESS, reg, 8)) != I2C_MASTER_ERR_NONE){
    CCS_I2C_disable();    // desactivate I2C
    return err | CCS811_ERR_READ_MEASURE<<8;
  }
  CCS_I2C_disable();    // desactivate I2C
  return CCS811_ERR_NONE;
}

/*!**********************************************************************************
 * \brief       Set humidity and temperature values for sensor correction
 *
 * \return      \ref CCS811_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
set_temp_hum(void)
{
  uint16_t err;
  uint8_t data[4];

  data[0] = locTemperature;
  data[1] = 0;
  data[2] = locHumidity;
  data[3] = 0;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_enable();    // activate I2C

  if((err = i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_ENV_DATA)) != I2C_MASTER_ERR_NONE){
    CCS_I2C_disable();    // desactivate I2C
    return err | CCS811_ERR_WRITE_REGISTER<<8;
  }
  if((err = i2c_burst_send(CCS811_SLAVE_ADDRESS, data, 4)) != I2C_MASTER_ERR_NONE){
    CCS_I2C_disable();    // desactivate I2C
    return err | CCS811_ERR_WRITE_ENV_DATA<<8;
  }
  CCS_I2C_disable();    // desactivate I2C
  return CCS811_ERR_NONE;
}

/*!**********************************************************************************
 * \brief         Set mode of the measurement.
 *
 * \param mode      Mode to be used.
 *
 * \return        \ref CCS811_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
set_mode(uint8_t mode)
{
  uint8_t data[2];
  uint8_t err;

  data[0] = CCS811_MEAS_MODE;
  data[1] = mode;
  CCS_I2C_enable();    // activate I2C

  if((err = i2c_burst_send(CCS811_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
    err |= CCS811_ERR_WRITE_MODE<<8;
    CCS_I2C_disable();    // desactivate I2C
    return err;
  }
  CCS_I2C_disable();    // desactivate I2C
  return CCS811_ERR_NONE;
}

/*!**********************************************************************************
 * \brief         Set baseline of the measurement.
 *
 * \param mode      Baseline to set to sensor (if 0, last readed value is written again)
 *
 * \return        \ref CCS811_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
set_baseline(uint16_t baseline)
{
  uint8_t data[3];
  uint8_t err;

  if(baseline == 0)
  {
    baseline = locBaseline;
  }
  data[0] = CCS811_BASELINE;
  data[1] = baseline >> 8;
  data[2] = baseline;
  CCS_I2C_enable();    // activate I2C

  if((err = i2c_burst_send(CCS811_SLAVE_ADDRESS, data, 3)) != I2C_MASTER_ERR_NONE){
    err |= CCS811_ERR_WRITE_MODE<<8;
    CCS_I2C_disable();    // desactivate I2C
    return err;
  }
  CCS_I2C_disable();    // desactivate I2C
  return CCS811_ERR_NONE;
}

/*!**********************************************************************************
 * \brief       Power on the sensor
 *
 *
 * \return      \ref CCS811_ERR_NONE if success
 ***********************************************************************************/
static int
power_on(void)
{
  uint8_t fwv,stat;
  uint16_t err;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  //-------------------------------------------------------------------------------
  // power on chip and activate multiplexer channel
  GPIO_SET_OUTPUT( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);
  GPIO_CLR_PIN( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);

//  GPIO_SET_OUTPUT( CCS811_RST_PORT_BASE, CCS811_RST_PIN_MASK);
//  GPIO_SET_PIN( CCS811_RST_PORT_BASE, CCS811_RST_PIN_MASK); // activate
  deep_sleep_ms(50, NO_GPIO_INTERRUPT, 0);

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_disable();
  if((err = pca9546_channel_enable(PCA_9546_CCS811_SEL_POS)) != PCA9546_ERR_NONE)
  {
    return err;
  }
  // consumption in sleep mode: tbd

  /* Wait power-up sequence (value from datasheet) 20 ms*/
  deep_sleep_ms(20, NO_GPIO_INTERRUPT, 0);
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);

  //-------------------------------------------------------------------------------
  // get chip ID for security
  fwv = 0x00;
  CCS_I2C_enable();    // activate I2C
  if((err = i2c_single_send(CCS811_SLAVE_ADDRESS,CCS811_HW_ID)) != I2C_MASTER_ERR_NONE){
    err |= CCS811_ERR_WRITE_REGISTER<<8;
    CCS_I2C_disable();    // desactivate I2C
    return err;
  }
  if((err = i2c_single_receive(CCS811_SLAVE_ADDRESS,&fwv)) != I2C_MASTER_ERR_NONE){
    err |= CCS811_ERR_READ_ID<<8;
    CCS_I2C_disable();    // desactivate I2C
    return err;
  }
  CCS_I2C_disable();
  if(fwv != CCS811_HW_ID_DATA)
  {
    return fwv | (CCS811_ERR_BAD_ID<<8);
  }
  //-------------------------------------------------------------------------------
  // set mode application (not bootloader mode)
  CCS_I2C_enable();    // activate I2C
  if((err = i2c_single_send(CCS811_SLAVE_ADDRESS,CCS811_APP_START)) != I2C_MASTER_ERR_NONE){
    err |= CCS811_ERR_APP_START<<8;
    CCS_I2C_disable();    // desactivate I2C
    return err;
  }
  CCS_I2C_disable();    // desactivate I2C
  deep_sleep_ms(100, NO_GPIO_INTERRUPT, 0);
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_enable();    // activate I2C
  i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_STATUS);
  i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
  CCS_I2C_disable();    // desactivate I2C
  //-------------------------------------------------------------------------------
  // set mode of measure
  CCS_I2C_enable();    // activate I2C
  if((err = set_mode(locModeOfMeasure)) != I2C_MASTER_ERR_NONE){
    err |= CCS811_ERR_WRITE_MODE<<8;
    CCS_I2C_disable();    // desactivate I2C
    return err;
  }
  CCS_I2C_disable();    // desactivate I2C
  deep_sleep_ms(5, NO_GPIO_INTERRUPT, 0);
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_enable();
  i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_STATUS);
  i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
  CCS_I2C_disable();
  deep_sleep_ms(5, NO_GPIO_INTERRUPT, 0);
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_enable();
  i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_ERROR_ID);
  i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
  CCS_I2C_disable();
  //-------------------------------------------------------------------------------
  // set last known baseline
//  CCS_I2C_disable();
//  if((err = set_baseline(locBaseline)) != I2C_MASTER_ERR_NONE){
//    err |= CCS811_ERR_SET_BASELINE<<8;
//    CCS_I2C_disable();    // desactivate I2C
//    return err;
//  }
//  deep_sleep_ms(5);
//  CCS_I2C_enable();
  i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_STATUS);
  i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
  CCS_I2C_disable();
  deep_sleep_ms(5, NO_GPIO_INTERRUPT, 0);
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  CCS_I2C_enable();
  i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_ERROR_ID);
  i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
  CCS_I2C_disable();    // desactivate I2C
  return CCS811_ERR_NONE;
}

/*!**********************************************************************************
 * \brief       Power off the sensor
 *
 *
 * \return      \ref CCS811_ERR_NONE if success
 ***********************************************************************************/
static int
power_off(void)
{
  uint16_t err;

  //-------------------------------------------------------------------------------
  // power off chip and desactivate multiplexer channel
  if((err = pca9546_channel_disable(PCA_9546_CCS811_SEL_POS)) != PCA9546_ERR_NONE)
  {
    GPIO_SET_OUTPUT( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);
    GPIO_SET_PIN( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);

    GPIO_SET_OUTPUT( CCS811_RST_PORT_BASE, CCS811_RST_PIN_MASK);
    GPIO_CLR_PIN( CCS811_RST_PORT_BASE, CCS811_RST_PIN_MASK); // activate
    CCS_I2C_enable();    // don't draw current (active low)
    return err;
  }
  GPIO_SET_OUTPUT( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);
  GPIO_SET_PIN( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);
  GPIO_SET_OUTPUT( CCS811_RST_PORT_BASE, CCS811_RST_PIN_MASK);
  GPIO_CLR_PIN( CCS811_RST_PORT_BASE, CCS811_RST_PIN_MASK); // activate
  CCS_I2C_enable();    // don't draw current (active low)
  return CCS811_ERR_NONE;
}

/*!**********************************************************************************
 * \brief       Read sensor value
 *
 * \param type    Select the kind of value to read.
 *
 * \return      The sensor value
 ***********************************************************************************/
static int
value(int type)
{
  switch(type) {
    case CCS811_SENSE_CO2:
      return co2;
    break;
    case CCS811_SENSE_TVOC:
      return tvoc;
    break;
  default:
    return ~CCS811_ERR_NONE;
  }
  return ~CCS811_ERR_NONE;
}

/*!************************************************************************************
 * \brief     Configure function provided by the sensors API
 *
 * \param type  Configuration type
 * \param value Used as binary value to change the state of the sensor
 *        or as a numerical value to configure a specific parameter.
 * \return    \ref SHT21_ERR_NONE in case of success or the error value
 ************************************************************************************/
static int
configure(int type, int value)
{
  uint16_t err;
  uint8_t data[8];
  uint8_t stat;

  switch(type) {
    case SENSORS_HW_INIT:
#if 1
      if((err=power_off()) != CCS811_ERR_NONE)
      {
        return err | (CCS811_ERR_POWER_OFF << 8);
      }
#endif
    break;
    case SENSORS_DO_MEASURE:
      // start measure one shot (mode WEATHER or SMARTSENSOR)
      if( (err = read_sensor(data)) != CCS811_ERR_NONE){
        return err | CCS811_ERR_READ_MEASURE<<8;
      }
      CCS_I2C_enable();
      i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_STATUS);
      i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
      CCS_I2C_disable();
      deep_sleep_ms(5, NO_GPIO_INTERRUPT, 0);
      i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
      CCS_I2C_enable();
      i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_ERROR_ID);
      i2c_single_receive(CCS811_SLAVE_ADDRESS, &stat);
      CCS_I2C_disable();    // desactivate I2C
    // TODO -> define duration of measure depending on mode ... ???
    //  ENERGEST_ON(ENERGEST_TYPE_SENSORS_CCS811);
    //  deep_sleep_ms(100);   // value for smartsensor configutation
    //              // TODO -> check usage mode for delay
    //  ENERGEST_OFF(ENERGEST_TYPE_SENSORS_CCS811);
      co2 =  ((uint16_t)data[0] << 8) | data[1];
      tvoc = ((uint16_t)data[2] << 8) | data[3];
      break;
    case SENSORS_ACTIVE:
      if(value == 1)
      {
        if(GPIO_READ_PIN( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK)){
          if((err=power_on()) != CCS811_ERR_NONE)
          {
            return err | (CCS811_ERR_POWER_ON << 8);
          }
        }
      }
      else
      {
        if((err=power_off()) != CCS811_ERR_NONE)
        {
          return err | (CCS811_ERR_POWER_OFF << 8);
        }
      }
    break;
    case CCS811_CURRENT_MODE:
      if( (err = set_temp_hum()) != CCS811_ERR_NONE){
        return err | CCS811_ERR_WRITE_ENV_DATA<<8;
      }
      if( (err = set_mode(value)) != CCS811_ERR_NONE){
        return err | CCS811_ERR_SET_MODE<<8;
      }
      break;
    case CCS811_CURRENT_BASELINE:
      if( (err = set_baseline(value)) != CCS811_ERR_NONE){
        return err | CCS811_ERR_SET_BASELINE<<8;
      }
      locBaseline = value;
      break;
    case CCS811_CURRENT_TEMPERATURE:
      locTemperature = (value * 2) / 10 + 50;  // value is in 10degree (ex. 22.5 deg = 225)
      break;
    case CCS811_CURRENT_HUMIDITY:
      locHumidity = (value * 2) / 10;  // value is in %% (ex. 45.5 % = 455)
      break;


  }
  return CCS811_ERR_NONE;
}

/*!************************************************************************************
 * \brief     Status function provided by the sensors API
 *
 * \param type  Type of status to return
 * \return    Value of the status
 ************************************************************************************/
static int
status(int type)
{
  int16_t err;
  uint8_t data;
  uint8_t tmp[2];

  switch(type) {

    case CCS811_POWER_STATE:
      return !GPIO_READ_PIN( CCS811_PWR_PORT_BASE, CCS811_PWR_PIN_MASK);
      break;

    case CCS811_CURRENT_STATUS:
      CCS_I2C_enable();    // activate I2C
      if((err = i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_STATUS)) != I2C_MASTER_ERR_NONE){
        CCS_I2C_disable();    // desactivate I2C
        return err | CCS811_ERR_WRITE_REGISTER<<8;
      }
      if((err = i2c_single_receive(CCS811_SLAVE_ADDRESS, &data)) != I2C_MASTER_ERR_NONE){
        CCS_I2C_disable();    // desactivate I2C
        return err | CCS811_ERR_READ_STATUS<<8;
      }
      CCS_I2C_disable();    // desactivate I2C
      return data;
    break;

    case CCS811_CURRENT_MODE:
      CCS_I2C_enable();    // activate I2C
      if((err = i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_MEAS_MODE)) != I2C_MASTER_ERR_NONE){
        CCS_I2C_disable();    // desactivate I2C
        return err | CCS811_ERR_WRITE_REGISTER<<8;
      }
      if((err = i2c_single_receive(CCS811_SLAVE_ADDRESS, &data)) != I2C_MASTER_ERR_NONE){
        CCS_I2C_disable();    // desactivate I2C
        return err | CCS811_ERR_READ_MODE<<8;
      }
      CCS_I2C_disable();    // desactivate I2C
      return data;
    break;

    case CCS811_CURRENT_BASELINE:
      CCS_I2C_enable();    // activate I2C
      if((err = i2c_single_send(CCS811_SLAVE_ADDRESS, CCS811_BASELINE)) != I2C_MASTER_ERR_NONE){
        CCS_I2C_disable();    // desactivate I2C
        return err | CCS811_ERR_WRITE_REGISTER<<8;
      }
      if((err = i2c_burst_receive(CCS811_SLAVE_ADDRESS, tmp, 2)) != I2C_MASTER_ERR_NONE){
        CCS_I2C_disable();    // desactivate I2C
        return err | CCS811_ERR_READ_BASELINE<<8;
      }
      CCS_I2C_disable();    // desactivate I2C
      locBaseline = ((uint16_t)tmp[0] << 8) | tmp[1];
      return locBaseline;
    break;
  }
  CCS_I2C_disable();    // desactivate I2C
  return ~CCS811_ERR_NONE;
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(ccs811_sensor, CCS811_SENSOR, value, configure, status);

/** @} */
