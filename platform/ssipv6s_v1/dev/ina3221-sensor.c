/**
 * \addtogroup sensors
 * @{
 *
 * \file
 * Driver for the Power sensor
 *
 * \author
 * Darko Petrovic
 */

#include "ina3221-sensor.h"

/** \cond */
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/** \endcond */

/*!**********************************************************************************
 * \brief 			Read chip register
 *
 * \param address 	Address of register to read
 * \param value 	Variable where to store the register value
 *
 * \return 			\ref INA3221_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
INA3221_read_register(uint8_t address, uint16_t *value)
{
	uint8_t data[2];
	uint16_t err;

	if( (err = i2c_single_send(INA3221_SLAVE_ADDRESS, address)) != I2C_MASTER_ERR_NONE )
	{
		return err | INA3221_ERR_SET_REGISTER<<8;
	}

	if ( (err = i2c_burst_receive(INA3221_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE ) {
		PRINTF("ina3221: (error) Fail to read register %#02x.\n", address);
		return err | INA3221_ERR_READ_REGISTER<<8;
	}

	*value = (data[0] << 8) + data[1];
	return INA3221_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Write chip register
 *
 * \param address 	Address of register to write
 * \param value 	Value to write
 *
 * \return 			\ref INA3221_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
INA3221_write_register(uint8_t address, uint16_t value)
{
	uint8_t data[3];
	uint16_t err;

	data[0] = address;
	data[1] = value >> 8;
	data[2] = value;

	if( (err = i2c_burst_send(INA3221_SLAVE_ADDRESS, data, 3)) != I2C_MASTER_ERR_NONE ){
		return err | INA3221_ERR_WRITE_REGISTER<<8;
	}
	return INA3221_ERR_NONE;
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
	uint8_t reg;	// register to read
	float sense_value, adjust;
	uint16_t data;
	static uint16_t return_value;
	uint16_t err;
	uint32_t mult;

	switch(type) {
		case INA3221_MEASUREMENT:

			PRINTF("ina3221: Measurement in progress.\n");

			// enable channels 1 and 2 only
			if ( (err = INA3221_write_register(INA3221_REG_CONF,
					INA3221_EN_CHANNEL_1 |
					INA3221_EN_CHANNEL_2 |
					INA3221_CONF_NB_AVG_4 |
					INA3221_MODE_SHUNT_BUS_VOLTAGE |
					INA3221_CONF_CT_BUS_140us |
					INA3221_CONF_CT_SHUNT_140us)) != I2C_MASTER_ERR_NONE )
			{
				return err | INA3221_ERR_EN_MEASURE<<8;
			}

			/* Use the code below to put the SoC in sleep mode while the INA3221 is performing a measurement.
			 * Put the SoC in sleep when the conversion time is too long. In any case we have to wait the measurement
			 * to finish before we get the value, otherwise we'll read the last measurement value. */
			return_value = 0;
			/* Read the Conversion Ready Flag */
			while( !(return_value & INA3221_MASKEN_CVRF) ){
				/* Adjust according 'Number of average' x 'Conversion time'
				 * e.g:  	1.1ms conv. time x 4 avg x 2 bus & shunt x 2 channels = 17.6ms
				 * 			0.140e-3 x 4 x 2 x 2 = 2.24 ms
				 */
				ENERGEST_ON(ENERGEST_TYPE_SENSORS_INA3221);
				deep_sleep_ms(3);
				ENERGEST_OFF(ENERGEST_TYPE_SENSORS_INA3221);
				i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
				if( (err = INA3221_read_register(INA3221_REG_MASK_ENABLE, &return_value)) != INA3221_ERR_NONE ){
					return err | INA3221_ERR_READ_REGISTER<<8;
				}

			}

			break;
		case INA3221_CH1_SHUNT_VOLTAGE:
			reg = INA3221_REG_CHAN1_SVOLT;
			adjust = INA3221_SHUNT_VOLT_LSB; // in V
			break;
		case INA3221_CH1_BUS_VOLTAGE:
			reg = INA3221_REG_CHAN1_BVOLT;
			adjust = INA3221_BUS_VOLT_LSB; // in V
			break;

		case INA3221_CH2_SHUNT_VOLTAGE:
			reg = INA3221_REG_CHAN2_SVOLT;
			adjust = INA3221_SHUNT_VOLT_LSB; // in V
			break;
		case INA3221_CH2_BUS_VOLTAGE:
			reg = INA3221_REG_CHAN2_BVOLT;
			adjust = INA3221_BUS_VOLT_LSB; // in V
			break;

		case INA3221_CH3_SHUNT_VOLTAGE:
			reg = INA3221_REG_CHAN3_SVOLT;
			adjust = INA3221_SHUNT_VOLT_LSB; // in V
			break;
		case INA3221_CH3_BUS_VOLTAGE:
			reg = INA3221_REG_CHAN3_BVOLT;
			adjust = INA3221_BUS_VOLT_LSB; // in V
			break;
	default:
		return ~INA3221_ERR_NONE;
	}

	if (type != INA3221_MEASUREMENT){
		// the measurement is done, get the values now
		if( (err = INA3221_read_register(reg, &data)) != INA3221_ERR_NONE ){
			return err | INA3221_ERR_READ_MEASURE<<8;
		}

		sense_value = (float)(((data>>3)&0x0FFF) * adjust);

		if( adjust < INA3221_BUS_VOLT_LSB ){
			mult = 1e5;
		} else {
			mult = 1e3;
		}

		return_value = (int)(sense_value*mult);
		ioc_set_over(I2C_SDA_PORT, I2C_SDA_PIN, IOC_OVERRIDE_PUE);
		ioc_set_over(I2C_SCL_PORT, I2C_SCL_PIN, IOC_OVERRIDE_PUE);
		return return_value;
	}
	return ~INA3221_ERR_NONE;

}

/*!************************************************************************************
 * \brief 		Configure function provided by the sensors API
 *
 * \param type 	Configuration type
 * \param value Used as binary value to change the state of the sensor
 * 				or as a numerical value to configure a specific parameter.
 * \return 		\ref INA3221_ERR_NONE in case of success otherwise the error value
 ************************************************************************************/
static int
configure(int type, int value)
{
	uint16_t err;

	switch(type) {
		case SENSORS_HW_INIT:
			i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
			// put the device in power down mode
			if( (err = INA3221_write_register(INA3221_REG_CONF, INA3221_MODE_POWER_DOWN )) != INA3221_ERR_NONE ){
				return err;
			}
			break;
		case SENSORS_ACTIVE:
			// power-up/down the sensor
			if( value ){
				i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
			} else {
				if( (err = INA3221_write_register(INA3221_REG_CONF, INA3221_MODE_POWER_DOWN )) != INA3221_ERR_NONE ){
					return err;
				}

			}

	}
	return INA3221_ERR_NONE;
}

/*!************************************************************************************
 * \brief 		Status function provided by the sensors API (not implemented here)
 *
 * \param type 	Type of status to return
 * \return 		Value of the status
 ************************************************************************************/
static int
status(int type)
{
  return 1;
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(ina3221_sensor, INA3221_SENSOR, value, configure, status);

/** @} */
