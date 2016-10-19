/**
 * \addtogroup sensor-bmp280
 * @{
 *
 * \file
 * Driver for the BMP280 sensor
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 */

#include "bmp280-sensor.h"
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

static volatile bmp280_cal_val_t calVal;
static uint8_t currentMode = BMP280_DEFAULT_MODE;
static int32_t temperature;
static uint32_t pressure;

/*!************************************************************************************
 * \brief 			Reads the factory setted calibration values
 *
 *
 * \return 			\ref BMP280_ERR_NONE in case of success otherwise error
 ************************************************************************************/
static uint16_t
get_calibration_values(void)
{
  uint8_t cal[BMP280_NB_CAL_REG];
  uint16_t err;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = i2c_single_send(BMP280_SLAVE_ADDRESS, BMP280_CALIB_0)) != I2C_MASTER_ERR_NONE){
    return err | BMP280_ERR_GET_CALIB<<8;
  }
  if((err = i2c_burst_receive(BMP280_SLAVE_ADDRESS, cal, BMP280_NB_CAL_REG)) != I2C_MASTER_ERR_NONE){
    return err | BMP280_ERR_READ_CALIB<<8;
  }
  calVal.dig_T1 = cal[0] | ((uint16_t)cal[1] << 8);
  calVal.dig_T2 = cal[2] | ((uint16_t)cal[3] << 8);
  calVal.dig_T3 = cal[4] | ((uint16_t)cal[5] << 8);
  calVal.dig_P1 = cal[6] | ((uint16_t)cal[7] << 8);
  calVal.dig_P2 = cal[8] | ((uint16_t)cal[9] << 8);
  calVal.dig_P3 = cal[10] | ((uint16_t)cal[11] << 8);
  calVal.dig_P4 = cal[12] | ((uint16_t)cal[13] << 8);
  calVal.dig_P5 = cal[14] | ((uint16_t)cal[15] << 8);
  calVal.dig_P6 = cal[16] | ((uint16_t)cal[17] << 8);
  calVal.dig_P7 = cal[18] | ((uint16_t)cal[19] << 8);
  calVal.dig_P8 = cal[20] | ((uint16_t)cal[21] << 8);
  calVal.dig_P9 = cal[22] | ((uint16_t)cal[23] << 8);
  calVal.tFine = 0;
  return BMP280_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Read temperature
 *
 * \param reg 		Location where to store the temperature
 *
 * \return 			\ref BMP280_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_temperature(int32_t *reg)
{
  uint8_t data[3];
  uint16_t err;
  int32_t temp32;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = i2c_single_send(BMP280_SLAVE_ADDRESS, BMP280_TEMP_MSB)) != I2C_MASTER_ERR_NONE){
    return err | BMP280_ERR_GET_TEMP<<8;
  }
  if((err = i2c_burst_receive(BMP280_SLAVE_ADDRESS, data, 3)) != I2C_MASTER_ERR_NONE){
    return err | BMP280_ERR_READ_TEMP<<8;
  }
  temp32 = data[0] << 12;
  temp32 = temp32 | data[1] << 4;
  temp32 = temp32 | data[2] >> 4;
  *reg = temp32;
  return BMP280_ERR_NONE;
}


/*!**********************************************************************************
 * \brief 			Read pressure
 *
 * \param reg 		Location where to store the pressure
 *
 * \return 			\ref BMP280_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_pressure(uint32_t *reg)
{
  uint8_t data[3];
  uint16_t err;
  uint32_t temp32;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = i2c_single_send(BMP280_SLAVE_ADDRESS, BMP280_PRESS_MSB)) != I2C_MASTER_ERR_NONE){
    return err | BMP280_ERR_GET_TEMP<<8;
  }
  if((err = i2c_burst_receive(BMP280_SLAVE_ADDRESS, data, 3)) != I2C_MASTER_ERR_NONE){
    return err | BMP280_ERR_READ_TEMP<<8;
  }

  temp32 = data[0] << 12;
  temp32 = temp32 | data[1] << 4;
  temp32 = temp32 | data[2] >> 4;
  *reg = temp32;
  return BMP280_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Compensate temperature
 *
 * \param v_uncomp_temperature_s32 	Uncompensed temperature
 *
 * \return 			Compensated temperature
 * 					The temperature is returned in 0.01 degree resolution
 * 					It means 25.43 degree returns 2543
 ***********************************************************************************/
static int32_t
bmp280_compensate_temperature_int32(int32_t v_uncomp_temperature_s32)
{
  int32_t v_x1_u32r = 0;
  int32_t v_x2_u32r = 0;
  int32_t temperature = 0;

  /* calculate true temperature*/
  /*calculate x1*/
  v_x1_u32r = ((((v_uncomp_temperature_s32 >> 3) - ((int32_t) calVal.dig_T1 << 1))) *
      ((int32_t)calVal.dig_T2)) >> 11;
  /*calculate x2*/
  v_x2_u32r = (((((v_uncomp_temperature_s32 >> 4) - ((int32_t)calVal.dig_T1)) *
      ((v_uncomp_temperature_s32 >> 4) - ((int32_t)calVal.dig_T1))) >> 12) *
      ((int32_t)calVal.dig_T3)) >> 14;
  /*calculate t_fine*/
  calVal.tFine = v_x1_u32r + v_x2_u32r;
  /*calculate temperature*/
  temperature = (calVal.tFine * 5 + 128) >> 8;
  return temperature;
}

/*!**********************************************************************************
 * \brief 			Compensate pressure
 *
 * \param v_uncomp_pressure_s32 	Uncompensed pressure
 *
 * \return 			Compensated pressure
 ***********************************************************************************/
static uint32_t
bmp280_compensate_pressure_int32(int32_t v_uncomp_pressure_s32)
{
  int32_t v_x1_u32r = 0;
  int32_t v_x2_u32r = 0;
  uint32_t v_pressure_u32 = 0;

  /* calculate x1*/
  v_x1_u32r = (((int32_t)calVal.tFine) >> 1) - (int32_t)64000;
  /* calculate x2*/
  v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *
      ((int32_t)calVal.dig_P6);
  v_x2_u32r = v_x2_u32r + ((v_x1_u32r *((int32_t)calVal.dig_P5)) << 1);
  v_x2_u32r = (v_x2_u32r >> 2) + (((int32_t)calVal.dig_P4) << 16);
  /* calculate x1*/
  v_x1_u32r = (((calVal.dig_P3 *(((v_x1_u32r >> 2) *
      (v_x1_u32r >> 2)) >> 13)) >> 3) + ((((int32_t)calVal.dig_P2) *
      v_x1_u32r) >> 1)) >> 18;
  v_x1_u32r = ((((32768 + v_x1_u32r)) *((int32_t)calVal.dig_P1)) >> 15);
  /* calculate pressure*/
  v_pressure_u32 = (((uint32_t)(((int32_t)1048576) - v_uncomp_pressure_s32) -
      (v_x2_u32r >> 12))) * 3125;
  /* check overflow*/
  if (v_pressure_u32 < 0x80000000)
  {
    /* Avoid exception caused by division by zero */
    if (v_x1_u32r != 0)
    {
      v_pressure_u32 =(v_pressure_u32 << 1) / ((uint32_t)v_x1_u32r);
    }
    else
    {
      return 0;	// invalid data
    }
  }
  else
  {
    /* Avoid exception caused by division by zero */
    if (v_x1_u32r != 0)
    {
      v_pressure_u32 = (v_pressure_u32 / (uint32_t)v_x1_u32r) * 2;
    }
    else
    {
      return 0;	// invalid data
    }
  }
  /* calculate x1*/
  v_x1_u32r = (((int32_t) calVal.dig_P9) *
        ((int32_t)(((v_pressure_u32 >> 3) *
        (v_pressure_u32 >> 3)) >> 13))) >> 12;
  /* calculate x2*/
  v_x2_u32r = (((int32_t)(v_pressure_u32 >> 2)) *
        ((int32_t)calVal.dig_P8)) >> 13;
  /* calculate true pressure*/
  v_pressure_u32 = (uint32_t) ((int32_t)v_pressure_u32 +
      ((v_x1_u32r + v_x2_u32r + calVal.dig_P7) >> 4));
  return v_pressure_u32;
}

/*!**********************************************************************************
 * \brief 				Set the resolution of the measurement.
 *
 * \param mode		 	Mode to be used.
 * \see					BMP280_DEFAULT_RES
 *
 * \return 				\ref BMP280_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
set_mode(uint8_t mode)
{
  uint8_t data[4];
  uint8_t err;

  switch(mode)
  {
    case BMP280_HANDHELD_LOW_POWER_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_4 | BMP280_STANDBY_TIME_63_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_16X | BMP280_OVERSAMPT_2X | BMP280_NORMAL_MODE;
    break;
    case BMP280_HANDHELD_DYNAMIC_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_16 | BMP280_STANDBY_TIME_1_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_4X | BMP280_OVERSAMPT_1X | BMP280_NORMAL_MODE;
    break;
    case BMP280_WEATHER_MONITORING_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_OFF | BMP280_STANDBY_TIME_1_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_1X | BMP280_OVERSAMPT_1X | BMP280_FORCED_MODE;
    break;
    case BMP280_ELEVATOR_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_4 | BMP280_STANDBY_TIME_125_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_4X | BMP280_OVERSAMPT_1X | BMP280_NORMAL_MODE;
    break;
    case BMP280_DROP_DETECT_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_OFF | BMP280_STANDBY_TIME_1_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_2X | BMP280_OVERSAMPT_1X | BMP280_NORMAL_MODE;
    break;
    case BMP280_INDOOR_NAVIGATION_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_16 | BMP280_STANDBY_TIME_1_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_16X | BMP280_OVERSAMPT_2X | BMP280_NORMAL_MODE;
    break;
    case BMP280_SMARTSENSOR_MODE:
      data[0] = BMP280_CONFIG;
      data[1] = BMP280_FILTER_COEFF_OFF | BMP280_STANDBY_TIME_1_MS;
      data[2] = BMP280_CTRL_MEAS;
      data[3] = BMP280_OVERSAMPP_16X | BMP280_OVERSAMPT_16X | BMP280_FORCED_MODE;
    break;
  }


  if((err = i2c_burst_send(BMP280_SLAVE_ADDRESS, data, 4)) != I2C_MASTER_ERR_NONE){
    err |= BMP280_ERR_WRITE_MODE<<8;
    return err;
  }
  currentMode = mode;
  return BMP280_ERR_NONE;
}
/*!**********************************************************************************
 * \brief 			Power on the sensor
 *
 *
 * \return 			\ref BMP280_ERR_NONE if success
 ***********************************************************************************/
static int
power_on(void)
{
  uint16_t err;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  //-------------------------------------------------------------------------------
  // power on chip and activate multiplexer channel
  GPIO_SET_PIN( BMP280_PWR_PORT_BASE, BMP280_PWR_PIN_MASK);
  if((err = pca9546_channel_enable(PCA_9546_BMP280_SEL_POS)) != PCA9546_ERR_NONE)
  {
    return err;
  }
  // consumption in sleep mode: tbd

  /* Wait power-up sequence (value from datasheet) 2 ms*/
  deep_sleep_ms(2, NO_GPIO_INTERRUPT, 0);
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  //-------------------------------------------------------------------------------
  // get chip ID for security
  /*
   uint8_t data;
   data = BMP280_ID;
  if((err = i2c_single_send(BMP280_SLAVE_ADDRESS,BMP280_ID)) != I2C_MASTER_ERR_NONE){
    err |= BMP280_ERR_WRITE_REGISTER<<8;
    return err;
  }
  if((err = i2c_single_receive(BMP280_SLAVE_ADDRESS,&data)) != I2C_MASTER_ERR_NONE){
    err |= BMP280_ERR_READ_ID<<8;
    return err;
  }
  if(data != BMP280_ID_DATA)
  {
    return data | (BMP280_ERR_BAD_ID<<8);
  }
  */
  //-------------------------------------------------------------------------------
  // set default accuracy mode
  if( (err = set_mode( currentMode )) != BMP280_ERR_NONE ){
    return err | BMP280_ERR_SET_MODE<<8;
  }
  return BMP280_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Power off the sensor
 *
 *
 * \return 			\ref BMP280_ERR_NONE if success
 ***********************************************************************************/
static int
power_off(void)
{
  uint16_t err;
  // power off chip and deactivate multiplexer channel
  err = pca9546_channel_disable(PCA_9546_BMP280_SEL_POS);
  GPIO_CLR_PIN( BMP280_PWR_PORT_BASE, BMP280_PWR_PIN_MASK);
  return err | BMP280_ERR_NONE<<8;
}

/*!**********************************************************************************
 * \brief 			Read sensor value
 *
 * \param type	 	Select the kind of value to read.
 *
 * \return 			The sensor value
 ***********************************************************************************/
static int
value(int type)
{
  switch(type) {
    case BMP280_TEMP:
      return temperature;
    break;
    case BMP280_PRESSURE:
      return pressure;
    break;
  default:
    return ~BMP280_ERR_NONE;
  }
  return ~BMP280_ERR_NONE;
}

/*!************************************************************************************
 * \brief 		Configure function provided by the sensors API
 *
 * \param type 	Configuration type
 * \param value Used as binary value to change the state of the sensor
 * 				or as a numerical value to configure a specific parameter.
 * \return 		\ref SHT21_ERR_NONE in case of success or the error value
 ************************************************************************************/
static int
configure(int type, int value)
{
  uint16_t err;

  switch(type) {
    case SENSORS_HW_INIT:
      GPIO_SET_OUTPUT( BMP280_PWR_PORT_BASE, BMP280_PWR_PIN_MASK);
      if((err=power_on()) != BMP280_ERR_NONE){
        return err | (BMP280_ERR_POWER_ON << 8);
      }
      // get calibration table
      if((err = get_calibration_values()) != BMP280_ERR_NONE){
        return err | (BMP280_ERR_GET_CALIB << 8);
      }
      if((err=power_off()) != BMP280_ERR_NONE){
        return err | (BMP280_ERR_POWER_OFF << 8);
      }
    break;

    case SENSORS_DO_MEASURE:
      // start measure one shot (mode WEATHER or SMARTSENSOR)
      if( (err = set_mode(currentMode)) != BMP280_ERR_NONE){
        return err | BMP280_ERR_SET_MODE<<8;
      }
      ENERGEST_ON(ENERGEST_TYPE_SENSORS_BMP280);
      deep_sleep_ms(100, NO_GPIO_INTERRUPT, 0);
      ENERGEST_OFF(ENERGEST_TYPE_SENSORS_BMP280);
      if((err = read_temperature(&temperature)) != BMP280_ERR_NONE){
        return err | BMP280_ERR_READ_TEMP<<8;
      }
      temperature = bmp280_compensate_temperature_int32(temperature);

      if((err = read_pressure(&pressure)) != BMP280_ERR_NONE){
        return err | BMP280_ERR_READ_PRESSURE<<8;
      }
      pressure = bmp280_compensate_pressure_int32(pressure);

      break;
    case SENSORS_ACTIVE:
      if(value == 1) {
        if((err=power_on()) != BMP280_ERR_NONE){
          return err | (BMP280_ERR_POWER_ON << 8);
        }
      } else {
        if((err=power_off()) != BMP280_ERR_NONE){
          return err | (BMP280_ERR_POWER_OFF << 8);
        }
      }
    break;

    case BMP280_ACCURACY_MODE:
      if( (err = set_mode(value)) != BMP280_ERR_NONE){
        return err | BMP280_ERR_SET_MODE<<8;
      }
      currentMode = value;
      break;
  }
  return BMP280_ERR_NONE;
}

/*!************************************************************************************
 * \brief 		Status function provided by the sensors API
 *
 * \param type 	Type of status to return
 * \return 		Value of the status
 ************************************************************************************/
static int
status(int type)
{
  int16_t err;
  uint8_t data;

  switch(type) {
    case BMP280_ACCURACY_MODE:
      return currentMode;
    break;

    case BMP280_CURRENT_STATUS:
      if((err = i2c_single_send(BMP280_SLAVE_ADDRESS, BMP280_STATUS)) != I2C_MASTER_ERR_NONE){
        return err | BMP280_ERR_WRITE_REGISTER<<8;
      }
      if((err = i2c_single_receive(BMP280_SLAVE_ADDRESS, &data)) != I2C_MASTER_ERR_NONE){
        return err | BMP280_ERR_READ_STATUS<<8;
      }
      return data;
    break;
  }
  return ~BMP280_ERR_NONE;
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(bmp280_sensor, BMP280_SENSOR, value, configure, status);

/** @} */
