/**
 * \addtogroup sensor-pca9546
 * @{
 *
 * \file
 * Driver for the PCA9546 multiplexer
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 */

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


static uint8_t pca9546_ctrl_reg = 0;

/*!**********************************************************************************
 * \brief 			Read control register
 *
 * \param reg 		Location where to store the register value
 *
 * \return 			\ref PCA9546_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
read_user_register(uint8_t *reg)
{
  uint8_t data;
  uint16_t err;

  if((err = i2c_single_receive(PCA9546_SLAVE_ADDRESS, &data)) != I2C_MASTER_ERR_NONE){
    return err | PCA9546_ERR_READ_CONTROL_REG<<8;
  }

  *reg = data;
  return PCA9546_ERR_NONE;
}

/*!**********************************************************************************
 * \brief 			Write control register
 *
 * \param value 	Value to write
 *
 * \return 			\ref PCA9546_ERR_NONE if success
 ***********************************************************************************/
static uint16_t
write_user_register(uint8_t value)
{
  uint16_t err;

  if((err = i2c_single_send(PCA9546_SLAVE_ADDRESS, value)) != I2C_MASTER_ERR_NONE){
    err |= PCA9546_ERR_WRITE_CONTROL_REG<<8;
    return err;
  }

  return PCA9546_ERR_NONE;
}


/*!************************************************************************************
 * \brief 		Resets the I2C multiplexer. Set all channel off.
 *
 * \return 		\ref PCA9546_ERR_NONE in case of success or the error value
 ************************************************************************************/
int
pca9546_init(void)
{
  GPIO_SET_OUTPUT( PCA9546_RST_PORT_BASE, PCA9546_RST_PIN_MASK);
  GPIO_CLR_PIN( PCA9546_RST_PORT_BASE, PCA9546_RST_PIN_MASK);
  clock_delay_usec(1);	// at least 500ns -> datasheet
  GPIO_SET_PIN( PCA9546_RST_PORT_BASE, PCA9546_RST_PIN_MASK);
  return PCA9546_ERR_NONE;
}

/*!************************************************************************************
 * \brief 		Activation a an I2C channel
 *
 * \param channel 	Channel to activate
 * \return 		\ref PCA9546_ERR_NONE in case of success or the error value
 ************************************************************************************/
int
pca9546_channel_enable(uint8_t channel)
{
  pca9546_ctrl_reg |= (1 << channel);
  return write_user_register(pca9546_ctrl_reg);
}

 /*!************************************************************************************
  * \brief 		Desactivation an I2C channel
  *
  * \param channel 	Channel to desactivate
  * \return 		\ref PCA9546_ERR_NONE in case of success or the error value
  ************************************************************************************/
int
pca9546_channel_disable(uint8_t channel)
{
  pca9546_ctrl_reg &= ~(1 << channel);
  return write_user_register(pca9546_ctrl_reg);
}


/** @} */
