/**
 * \addtogroup sensors
 * @{
 *
 * \file
 * Driver for the SHT21 sensor
 *
 * \author
 * Darko Petrovic
 */

#include "sht21-sensor.h"

/** \cond */
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/** \endcond */

/*!************************************************************************************
 * \brief 			Compute and compare CRC
 *
 * \param data 		Array of data on which CRC is performed
 * \param dataSize 	Size of data
 * \param chksm 	Checksum to be compared
 *
 * \return 			\ref SHT21_ERR_NONE in case of success otherwise \ref SHT21_ERR_CRC_CHECK
 ************************************************************************************/
static uint8_t
check_crc_SHT21(uint8_t data[], uint8_t dataSize, uint8_t chksm)
{
	uint8_t crc=0, i, j;
	for(i=0; i<dataSize; ++i) {
		crc ^= data[i];
		for(j=8; j>0; --j) {
			if(crc & 0x80) {
				crc = (crc<<1) ^ SHT21_CRC_POLYNOMIAL;
			} else {
				crc = (crc<<1);
			}
		}
	}
	if(crc != chksm) {
		return SHT21_ERR_CRC_CHECK;
	} else {
		return SHT21_ERR_NONE;
	}
}

/*!**********************************************************************************
 * \brief 			Read user register
 *
 * \param reg 		Location where to store the register value
 *
 * \return 			\ref SHT21_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_user_register(uint8_t *reg)
{
	uint8_t data[2];
	uint16_t err;

	if((err = i2c_single_send(SHT21_SLAVE_ADDRESS, SHT21_CMD_USER_READ)) != I2C_MASTER_ERR_NONE){
		return err | SHT21_ERR_CMD_USER_REG<<8;
	}

	if((err = i2c_burst_receive(SHT21_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
		return err | SHT21_ERR_READ_USER_REG<<8;
	}

	if((err = check_crc_SHT21(&data[0], 1, data[1])) != SHT21_ERR_NONE){
		return err<<8;
	}

	*reg = data[0];
	return SHT21_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Write user register
 *
 * \param value 	Value to write
 *
 * \return 			\ref SHT21_ERR_NONE if success
 ***********************************************************************************/
static uint8_t
write_user_register(uint8_t value)
{
	uint8_t data[2];
	uint8_t err;

	data[0] = SHT21_CMD_USER_WRITE;
	data[1] = value;

	if((err = i2c_burst_send(SHT21_SLAVE_ADDRESS, data, 2)) != I2C_MASTER_ERR_NONE){
		err |= SHT21_ERR_WRITE_USER_REG<<8;
		return err;
	}

	return SHT21_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 				Set the resolution of the measurement.
 *
 * \param resolution 	Resolution to be used.
 * \see	SHT21_DEFAULT_RES
 *
 * \return 				\ref SHT21_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
set_resolution(uint8_t resolution)
{
	uint8_t user_register;
	uint16_t err;

	/* Set the resolution.
	 * As described in the datasheet §5.6, the reserved bits of the user register
	 * may vary over time and must not be changed when writing the register. */
	if( (err = read_user_register(&user_register)) != SHT21_ERR_NONE ){
		return err;
	}

	switch( resolution ){
	case 0:
		resolution = SHT21_RES_RH12_T14;
		break;
	case 1:
		resolution = SHT21_RES_RH8_T12;
		break;
	case 2:
		resolution = SHT21_RES_RH10_T13;
		break;
	case 3:
		resolution = SHT21_RES_RH11_T11;
		break;
	}

	// set the resolution
	user_register = (user_register & ~SHT21_RES_MASK) | resolution;
	if( (err = write_user_register(user_register)) != SHT21_ERR_NONE ){
		return err;
	}
	return SHT21_ERR_NONE;
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
	uint8_t data[3];
	uint8_t cmd;
	float a, b;
	uint16_t err;
	uint8_t max_measurement_time;

	switch(type) {
		case SHT21_SENSE_TEMP:
			cmd = SHT21_CMD_MEAS_T | SHT21_MODE;
			a=175.72;
			b=46.85;
			max_measurement_time = 85;
			break;
		case SHT21_SENSE_HUMIDITY:
			cmd = SHT21_CMD_MEAS_RH | SHT21_MODE;
			a=125.0;
			b=6.0;
			max_measurement_time = 29;
			break;
	default:
		return ~SHT21_ERR_NONE;
	}

	if( (err = i2c_single_send(SHT21_SLAVE_ADDRESS, cmd)) == I2C_MASTER_ERR_NONE)
	{
		/* 	From the datasheet: Whenever the sensor
			is  powered  up,  but  not  performing  a  measurement  or
			communicating,  it  is  automatically  in  idle  state  (sleep
			mode).
		*/

#if SHT21_NO_HOLD_MODE
		/* The sensor pull-down the SCL line during measurment.
		 * We set the SCL pin as normal GPIO and wait for the interrupt at
		 * the end of the measurement when the sensor release the line. */
		clock_delay_usec(5000);
		// give the control of the pin back to the software
		i2c_master_disable();
		GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(I2C_SCL_PORT), GPIO_PIN_MASK(I2C_SCL_PIN));
		GPIO_DETECT_EDGE(GPIO_PORT_TO_BASE(I2C_SCL_PORT), GPIO_PIN_MASK(I2C_SCL_PIN));
		GPIO_TRIGGER_SINGLE_EDGE(GPIO_PORT_TO_BASE(I2C_SCL_PORT), GPIO_PIN_MASK(I2C_SCL_PIN));
		GPIO_POWER_UP_ON_RISING(I2C_SCL_PORT, GPIO_PIN_MASK(I2C_SCL_PIN));
		GPIO_ENABLE_POWER_UP_INTERRUPT(I2C_SCL_PORT, GPIO_PIN_MASK(I2C_SCL_PIN));
		nvic_interrupt_enable(NVIC_INT_GPIO_PORT_A);

		// sleep the SoC and wait interrupt on the SCL pin
		ENERGEST_ON(ENERGEST_TYPE_SENSORS_SHT21);
		deep_sleep_ms( max_measurement_time );
		ENERGEST_OFF(ENERGEST_TYPE_SENSORS_SHT21);

		GPIO_DISABLE_POWER_UP_INTERRUPT(I2C_SCL_PORT, GPIO_PIN_MASK(I2C_SCL_PIN));
		nvic_interrupt_disable(NVIC_INT_GPIO_PORT_A);
		// give the pin control back to the I2C module
		GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(I2C_SCL_PORT), GPIO_PIN_MASK(I2C_SCL_PIN));
		ioc_set_over(I2C_SCL_PORT, I2C_SCL_PIN, IOC_OVERRIDE_PUE);
		REG(IOC_I2CMSSCL) = ioc_input_sel(I2C_SCL_PORT, I2C_SCL_PIN );
		ioc_set_sel(I2C_SCL_PORT, I2C_SCL_PIN, IOC_PXX_SEL_I2C_CMSSCL);
		i2c_master_enable();
#endif

		//clock_delay_msec(delay);
		// Read 3 bytes: MSB, LSB and CRC
		if( (err = i2c_burst_receive(SHT21_SLAVE_ADDRESS, data, 3)) == I2C_MASTER_ERR_NONE ){
			if( (err = check_crc_SHT21(&data[0], 2, data[2])) == SHT21_ERR_NONE ){
				sense_value = ( (data[0]<<8) | (data[1]&~0x0003) );
				sense_value = ((((float)sense_value)/65536)*a-b)*100;
				return sense_value;
			} else {
				return err<<8;
			}
		} else {
			return err | SHT21_ERR_READ_MEASURE<<8;
		}
	} else {
		return err | SHT21_ERR_EN_MEASURE<<8;
	}
	return ~SHT21_ERR_NONE;

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
	uint8_t user_register;
	uint16_t err;

	switch(type) {
		case SENSORS_HW_INIT:
			GPIO_SET_OUTPUT( SHT21_PWR_PORT_BASE, SHT21_PWR_PIN_MASK);
			GPIO_SET_PIN( SHT21_PWR_PORT_BASE, SHT21_PWR_PIN_MASK);
			// consumption in sleep mode: 0.15uA (typ) to 0.4uA
			// the chip goes automatically in sleep mode after a power on reset
			// and when no performing measurement

			/* Wait power-up sequence (value from datasheet) */
			clock_delay_usec(15000);
			/*rtimer_arch_schedule( RTIMER_NOW() + RTIMER_SECOND*15 );
			REG(SYS_CTRL_PMCTL) = SYS_CTRL_PMCTL_PM2;
			do { asm("wfi"::); } while(0);*/

			if( (err = set_resolution( SHT21_DEFAULT_RES )) != SHT21_ERR_NONE ){
				return err | SHT21_ERR_SET_RESOLUTION<<8;
			}

			break;
		case SENSORS_ACTIVE:
			/* Nothing to do, the sensors goes automatically in sleep-mode when doing nothing. */
			i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);
			break;

		case SHT21_RESOLUTION:
			if( (err = set_resolution(value)) != SHT21_ERR_NONE){
				return err | SHT21_ERR_SET_RESOLUTION<<8;
			}
			break;

		case SHT21_HEATER:
			if((err = read_user_register(&user_register)) != SHT21_ERR_NONE ){
				return err | SHT21_ERR_SET_HEATER<<8 ;
			}

			if( value ){
				user_register |= SHT21_HEATER_ON;
			} else {
				user_register = (user_register & ~SHT21_HEATER_MASK) | SHT21_HEATER_OFF;
			}

			if( (err = write_user_register(user_register)) != SHT21_ERR_NONE ){
				return err | SHT21_ERR_SET_HEATER<<8 ;
			}
			break;
	}
	return SHT21_ERR_NONE;
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
	uint8_t user_register;
	int16_t err;

	switch(type) {
	case SHT21_RESOLUTION:
		if( (err = read_user_register(&user_register)) != SHT21_ERR_NONE ){
			return err;
		}

		switch( user_register & SHT21_RES_MASK ){
		case SHT21_RES_RH12_T14:
			return 0;
			break;
		case SHT21_RES_RH8_T12:
			return 1;
			break;
		case SHT21_RES_RH10_T13:
			return 2;
			break;
		case SHT21_RES_RH11_T11:
			return 3;
			break;
		}
		break;

	case SHT21_HEATER:
		if( (err = read_user_register(&user_register)) != SHT21_ERR_NONE ){
			return err;
		}
		return user_register & SHT21_HEATER_MASK;
		break;
	}

	return ~SHT21_ERR_NONE;
}

/** Instantiation of the sensor object in memory. */
SENSORS_SENSOR(sht21_sensor, SHT21_SENSOR, value, configure, status);

/** @} */
