/**
 * \addtogroup sensors
 * @{
 *
 * \defgroup sensor-pca9546 PCA9546 multiplexor Driver
 * @{
 *
 * \file
 * Header file for PCA9546 multiplexor driver
 *
 * \author
 * Darko Petrovic / Pascal Sartoretti
 *
 */


#ifndef PCA9546_MUX_H_
#define PCA9546_MUX_H_

#include "lib/sensors.h"
#include "i2c.h"
#include "platform-sensors.h"
#include <stdint.h>

#define PCA9546_MUX             "PCA9546"

#define PCA9546_RST_PORT_BASE   GPIO_PORT_TO_BASE(PCA9546_RST_PORT)
#define PCA9546_RST_PIN_MASK    GPIO_PIN_MASK(PCA9546_RST_PIN)

/**
 * \name Error codes
 *
 * \note First byte in error code printed in debug output.
 *
 * @{
 */
#define PCA9546_ERR_NONE                0x00
#define PCA9546_ERR_READ_CONTROL_REG    0x01
#define PCA9546_ERR_WRITE_CONTROL_REG   0x02

/** @} */

/*!************************************************************************************
 * \brief 		Resets the I2C multiplexer. Set all channel off.
 *
 * \return 		\ref PCA9546_ERR_NONE in case of success or the error value
 ************************************************************************************/
int
pca9546_init(void);
/*!************************************************************************************
 * \brief 		Activation a an I2C channel
 *
 * \param channel 	Channel to activate
 * \return 		\ref PCA9546_ERR_NONE in case of success or the error value
 ************************************************************************************/
int
pca9546_channel_enable(uint8_t channel);
 /*!************************************************************************************
 * \brief 		Desactivation a an I2C channel
 *
 * \param channel 	Channel to desactivate
 * \return 		\ref PCA9546_ERR_NONE in case of success or the error value
 ************************************************************************************/
int
pca9546_channel_disable(uint8_t channel);

#endif /* PCA9546_MUX_H_ */

/**
 * @}
 * @}
 */
