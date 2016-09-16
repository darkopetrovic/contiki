/**
 * \addtogroup sensors
 * @{
 *
 * \file
 * Driver for the back temperature sensor
 *
 * \author
 * Darko Petrovic
 */

#include "tmp100-sensor.h"

/** Store the resolution configured by the user to be retrieved later. */
static uint8_t resolution;

/*!**********************************************************************************
 * \brief 			Write config register
 *
 * \param value 	Value to write
 *
 * \return 			\ref TMP100_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
write_config_register(uint8_t value)
{
	uint8_t reg[2];
	uint16_t err;

	i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
	reg[0] = TMP100_REG_CONFIG;	// pointer to the config register
	reg[1] = value;  // value for the register
	if( (err = i2c_burst_send(TMP100_SLAVE_ADDRESS, reg, 2)) != I2C_MASTER_ERR_NONE )
	{
		return err | TMP100_ERR_WRITE_CONFIG<<8;
	}
	return TMP100_ERR_NONE;
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
	int16_t sense_value;
	uint8_t data[2];
	uint8_t reg;
	uint16_t err;
	float celcius;

	reg = TMP100_REG_TEMP;	// pointer to the temperature register

	if( type == TMP100_SENSE_TEMP ){
		// reconfigure I2C because of PM2 mode
		i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
		// write Pointer Register to Temperature register
		if( (err = i2c_single_send(TMP100_SLAVE_ADDRESS, reg)) != I2C_MASTER_ERR_NONE){
			return err | TMP100_ERR_WRITE_PR<<8;
		}
		// get measurement
		if( (err = i2c_burst_receive(TMP100_SLAVE_ADDRESS, data, 2)) == I2C_MASTER_ERR_NONE ){
			sense_value = ((data[0] << 8) | data[1]) >> 4;
			celcius = sense_value*0.0625;
			return celcius*100;
		} else {
			return err | TMP100_ERR_READ_MEASURE<<8;
		}
	}
	return ~TMP100_ERR_NONE;
}

/*!************************************************************************************
 * \brief 		Configure function provided by the sensors API
 *
 * \param type 	Configuration type
 * \param value Used as binary value to change the state of the sensor
 * 				or as a numerical value to configure a specific parameter.
 * \return 		\ref TMP100_ERR_NONE in case of success otherwise the error value
 ************************************************************************************/
static int
configure(int type, int value)
{
	uint32_t conversion_time_ms;
	uint16_t err;

	switch(type) {
		case SENSORS_HW_INIT:
			GPIO_SET_OUTPUT( TMP100_PWR_PORT_BASE, TMP100_PWR_PIN_MASK);
			GPIO_SET_PIN( TMP100_PWR_PORT_BASE, TMP100_PWR_PIN_MASK);

			/* put the device in sleep mode (writing 1 to the SD bit in config register)
			 * consumption in sleep mode: 0.1uA (typ) to 1uA */

			/* Wait power-up sequence (no value specified in datasheet but wait anyway) */
			deep_sleep_ms(15);

			if( (err = write_config_register( TMP100_SHUTDOWN )) != TMP100_ERR_NONE ){
				return err;
			}

			resolution = TMP100_DEFAULT_RES<<5;
			break;

		case SENSORS_ACTIVE:
			// Perform one-shot measurement at sensor activation
			if( value ){
				// one-shot conversion by setting OS/ALERT bit
				if( (err = write_config_register( TMP100_SHUTDOWN | TMP100_OS_ALERT | resolution )) != TMP100_ERR_NONE ){
					return err | TMP100_ERR_EN_MEASURE<<8;
				}

				switch( resolution ){
				case TMP100_RES_9bits:
					conversion_time_ms = TMP100_CONVERSION_TIME_BASE;
					break;
				case TMP100_RES_10bits:
					conversion_time_ms = TMP100_CONVERSION_TIME_BASE*2;
					break;
				case TMP100_RES_11bits:
					conversion_time_ms = TMP100_CONVERSION_TIME_BASE*4;
					break;
				case TMP100_RES_12bits:
					conversion_time_ms = TMP100_CONVERSION_TIME_BASE*8;
					break;
				default:
					conversion_time_ms = TMP100_CONVERSION_TIME_BASE*8;
				}

				// wait here during measurement
				ENERGEST_ON(ENERGEST_TYPE_SENSORS_TMP100);
				deep_sleep_ms(conversion_time_ms);
				ENERGEST_OFF(ENERGEST_TYPE_SENSORS_TMP100);

			} else {
				if( (err = write_config_register( TMP100_SHUTDOWN )) != TMP100_ERR_NONE ){
					return err;
				}
			}
			break;
		case TMP100_RESOLUTION:
			resolution = value<<5;
			break;
	}

	return TMP100_ERR_NONE;
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
	switch( type ){
	case TMP100_RESOLUTION:
		return resolution;
		break;
	}

	return ~TMP100_ERR_NONE;
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(tmp100_sensor, TMP100_SENSOR, value, configure, status);

/** @} */
