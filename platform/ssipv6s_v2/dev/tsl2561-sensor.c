/**
 * \addtogroup sensor-tsl2561
 * @{
 *
 * \file
 * Driver for the TSL2561 sensor
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 */

#include "tsl2561-sensor.h"
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

static uint8_t currentMode = TSL2561_G1_T402;
static uint16_t channel0 = 0;
static uint16_t channel1 = 0;
static uint32_t lux;

/*!**********************************************************************************
 * \brief 			Read visible light (and IR)
 *
 * \param reg 		Location where to store the visible (and IR) light
 *
 * \return 			\ref TSL2561_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_light_visible(uint16_t *reg)
{
  uint8_t data[2];
  uint16_t err;
  uint8_t command = TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = i2c_single_send(TSL2561_SLAVE_ADDRESS, command)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_CMD_SEND<<8;
  }
  if((err = i2c_burst_receive(TSL2561_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_MEASURE_READ<<8;
  }
  *reg = data[0] | (data[1] << 8);
  return TSL2561_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Read infrared light
 *
 * \param reg 		Location where to store the infrared light
 *
 * \return 			\ref TSL2561_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_light_ir(uint16_t *reg)
{
  uint8_t data[2];
  uint16_t err;
  uint8_t command = TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW;


  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = i2c_single_send(TSL2561_SLAVE_ADDRESS, command)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_CMD_SEND<<8;
  }
  if((err = i2c_burst_receive(TSL2561_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_MEASURE_READ<<8;
  }
  *reg = data[0] | (data[1] << 8);
  return TSL2561_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Returns the light measure in lux
 *
 * \param v_uncomp_temperature_s32 	Uncompensed temperature
 *
 * \return 			Compensated temperature
 * 					The temperature is returned in 0.01 degree resolution
 * 					It means 25.43 degree returns 2543
 ***********************************************************************************/
static uint32_t
light_lux(uint16_t ch0, uint16_t ch1)
{
  uint8_t tsl2561_gain;
  uint8_t tsl2561_integ;
  uint32_t chScale;
  uint32_t channel1 = (uint32_t) ch1;
  uint32_t channel0 = (uint32_t) ch0;
  uint32_t ratio = 0;
  uint16_t b, m;
  uint8_t data;
  uint16_t err;
  uint8_t command = TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING;

  if((err = i2c_single_send(TSL2561_SLAVE_ADDRESS, command)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_CMD_SEND<<8;
  }
  if((err = i2c_single_receive(TSL2561_SLAVE_ADDRESS, &data)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_TIMING_READ<<8;
  }

  tsl2561_gain = data & 0x10;
  tsl2561_integ = data & 0x03;
  switch(tsl2561_integ)
  {
    case TSL2561_INTEG_13_7MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
    break;
    case TSL2561_INTEG_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
    break;
    case TSL2561_INTEG_402MS:
      chScale = (1 << TSL2561_LUX_CHSCALE);
    break;
    default:
      return 0; //can't handle manual integration time here
    break;
  }
  if(!tsl2561_gain)
  {
    chScale = chScale << 4;
  }
  channel0 = (channel0 * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (channel1 * chScale) >> TSL2561_LUX_CHSCALE;
  if (channel0 != 0)
  {
    ratio = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;
  }
  ratio = (ratio + 1) >> 1;
  if((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if(ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if(ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if(ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if(ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if(ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if(ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if(ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
  uint32_t temp = (channel0 * b) - (channel1 * m);
  if(temp < 0) temp = 0;
  temp += (1 << (TSL2561_LUX_LUXSCALE - 1));
  return (temp >> TSL2561_LUX_LUXSCALE);
}


/*!**********************************************************************************
 * \brief 				Set the mode of the measurement.
 *
 * \param mode		 	Mode to be used.
 *
 * \return 				\ref TSL2561_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
set_mode(uint8_t mode)
{
  uint8_t err;

  uint8_t data[2];
  data[0] = TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_TIMING;
  data[1] = mode;

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = i2c_burst_receive(TSL2561_SLAVE_ADDRESS, data,2)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_SET_MODE<<8;
  }
  currentMode = mode;
  return TSL2561_ERR_NONE;
}
/*!**********************************************************************************
 * \brief 			Power on the sensor
 *
 *
 * \return 			\ref TSL2561_ERR_NONE if success
 ***********************************************************************************/
static int
power_on(void)
{
  uint16_t err;
  uint8_t data[2];

  // power on chip and activate multiplexer channel
  GPIO_SET_PIN( TSL2561_PWR_PORT_BASE, TSL2561_PWR_PIN_MASK);
  deep_sleep_ms(5, NO_GPIO_INTERRUPT, 0); // for test because datasheet don't explain it !
  // consumption in sleep mode: 3.2 to 15 uA

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
  if((err = pca9546_channel_enable(PCA_9546_TSL2561_SEL_POS)) !=PCA9546_ERR_NONE){
    return err;
  }

  // the interrupt pin needs a pull-up
  ioc_set_over(TSL2561_INT_PORT, TSL2561_INT_PIN, IOC_OVERRIDE_PUE);

  // enable interrupt to occur after every integration cycle
  data[0] = TSL2561_COMMAND_BIT | TSL2561_REGISTER_INTERRUPT;
  data[1] = 0x10;
  if((err = i2c_burst_send(TSL2561_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_CMD_SEND<<8;
  }

  //-------------------------------------------------------------------------------
  // get chip ID for security
  /*	uint8_t command = TSL2561_COMMAND_BIT | TSL2561_REGISTER_ID;


  if((err = i2c_single_send(TSL2561_SLAVE_ADDRESS, command)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_CMD_SEND<<8;
  }
  if((err = i2c_single_receive(TSL2561_SLAVE_ADDRESS, &data)) != I2C_MASTER_ERR_NONE){
    return err | TSL2561_ERR_ID_READ<<8;
  }
  if((data & 0xF0) != TSL2561_ID_DATA)
  {
    return data | (TSL2561_ERR_BAD_ID<<8);
  }
  */
  //-------------------------------------------------------------------------------
  // set default mode
  if( (err = set_mode( currentMode )) != TSL2561_ERR_NONE ){
    return err | TSL2561_ERR_SET_MODE<<8;
  }
  return TSL2561_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Power off the sensor
 *
 * \return 			\ref TSL2561_ERR_NONE if success
 ***********************************************************************************/
static int
power_off(void)
{
  uint16_t err;

  // deactivate multiplexer channel and power off the chip
  err = pca9546_channel_disable(PCA_9546_TSL2561_SEL_POS);

  // disable pull-up to not power on the chip through this pin
  ioc_set_over(TSL2561_INT_PORT, TSL2561_INT_PIN, IOC_OVERRIDE_PDE);
  GPIO_CLR_PIN( TSL2561_PWR_PORT_BASE, TSL2561_PWR_PIN_MASK);
  return err | TSL2561_ERR_NONE<<8;
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
    case TSL2561_LUX:
      return lux;
    break;
    case TSL2561_IR:
      return channel1;
    break;
    case TSL2561_VISIBLE:
      return channel0;
    break;
  default:
    return ~TSL2561_ERR_NONE;
  }
  return ~TSL2561_ERR_NONE;
}

/*!************************************************************************************
 * \brief 		Configure function provided by the sensors API
 *
 * \param type 	Configuration type
 * \param value Used as binary value to change the state of the sensor
 * 				or as a numerical value to configure a specific parameter.
 * \return 		\ref TSL2561_ERR_NONE in case of success or the error value
 ************************************************************************************/
static int
configure(int type, int value)
{
  uint16_t err;
  uint8_t data[2];

  switch(type) {
    case SENSORS_HW_INIT:
      GPIO_SET_INPUT( GPIO_PORT_TO_BASE(TSL2561_INT_PORT), GPIO_PIN_MASK(TSL2561_INT_PIN));
      ioc_set_over(TSL2561_INT_PORT, TSL2561_INT_PIN, IOC_OVERRIDE_PDE);
      GPIO_SET_OUTPUT( TSL2561_PWR_PORT_BASE, TSL2561_PWR_PIN_MASK);
      GPIO_CLR_PIN( TSL2561_PWR_PORT_BASE, TSL2561_PWR_PIN_MASK);
      break;

    case SENSORS_DO_MEASURE:
      data[0] = TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL;
      data[1] = 0x03;	// turn sensor active

      i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
      if((err = i2c_burst_send(TSL2561_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
        return err | TSL2561_ERR_ACTIVATE<<8;
      }

      ENERGEST_ON(ENERGEST_TYPE_SENSORS_TSL2561);
      deep_sleep_ms(430, TSL2561_INT_PORT, TSL2561_INT_PIN);
      ENERGEST_OFF(ENERGEST_TYPE_SENSORS_TSL2561);

      // disable chip
      data[0] = TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL;
      data[1] = 0x00;	// turn sensor off

      i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
      if((err=read_light_visible(&channel0)) != TSL2561_ERR_NONE){
        return err | TSL2561_ERR_LIGHT<<8;
      }
      if((err=read_light_ir(&channel1)) != TSL2561_ERR_NONE){
        return err | TSL2561_ERR_LIGHT<<8;
      }
      lux = light_lux(channel0, channel1);
      break;

    case SENSORS_ACTIVE:
      if(value == 1){
        if((err=power_on()) != TSL2561_ERR_NONE){
          return err | (TSL2561_ERR_POWER_ON << 8);
        }
      } else {
        if((err=power_off()) != TSL2561_ERR_NONE){
          return err | (TSL2561_ERR_POWER_OFF << 8);
        }
      }
      break;

    case TSL2561_MODE:
      if( (err = set_mode(value)) != TSL2561_ERR_NONE){
        return err | TSL2561_ERR_SET_MODE<<8;
      }
      currentMode = value;
      break;
  }
  return TSL2561_ERR_NONE;
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
  switch(type) {
    case TSL2561_CUR_MODE:
      return currentMode;
    break;

  }
  return ~TSL2561_ERR_NONE;
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(tsl2561_sensor, TSL2561_SENSOR, value, configure, status);

/** @} */
