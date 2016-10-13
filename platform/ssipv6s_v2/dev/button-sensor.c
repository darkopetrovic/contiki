/**
 * \addtogroup sensor-buttons
 * @{
 *
 * \file
 * Driver for the buttons
 *
 * \author
 * Darko Petrovic
 */

#include "button-sensor.h"

/** \cond */
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/** \endcond */

#define BUTTON_USER_PORT_BASE    	GPIO_PORT_TO_BASE(BUTTON_USER_PORT)
#define BUTTON_USER_PIN_MASK    	GPIO_PIN_MASK(BUTTON_USER_PIN)

#define USB_PLUG_DETECT_PORT_BASE	GPIO_PORT_TO_BASE(USB_PLUG_DETECT_PORT)
#define USB_PLUG_DETECT_PIN_MASK	GPIO_PIN_MASK(USB_PLUG_DETECT_PIN)

/*---------------------------------------------------------------------------*/
static struct timer debouncetimer;


/**
 * \brief Common initialiser for all buttons
 * \param port_base GPIO port's register offset
 * \param pin_mask Pin mask corresponding to the button's pin
 */
static void
config(uint32_t port_base, uint32_t pin_mask)
{
  /* Software controlled */
  GPIO_SOFTWARE_CONTROL(port_base, pin_mask);

  /* Set pin to input */
  GPIO_SET_INPUT(port_base, pin_mask);

  /* Enable edge detection */
  GPIO_DETECT_EDGE(port_base, pin_mask);

  /* Single edge */
  GPIO_TRIGGER_SINGLE_EDGE(port_base, pin_mask);

  /* Trigger interrupt on Rising edge */
  GPIO_DETECT_RISING(port_base, pin_mask);

  GPIO_ENABLE_INTERRUPT(port_base, pin_mask);
}


/**
 * \brief Callback registered with the GPIO module. Gets fired with a button
 * port/pin generates an interrupt
 * \param port The port number that generated the interrupt
 * \param pin The pin number that generated the interrupt. This is the pin
 * absolute number (i.e. 0, 1, ..., 7), not a mask
 */
static void
btn_callback(uint8_t port, uint8_t pin)
{
  if(!timer_expired(&debouncetimer)) {
    return;
  }

  PRINTF("Button callback\n");

  timer_set(&debouncetimer, CLOCK_SECOND / 8);

  if(port == GPIO_D_NUM) {
    switch(pin) {
      case BUTTON_USER_PIN:
        sensors_changed(&button_user_sensor);
        break;
      default:
        return;
    }
  }

  else if(port == GPIO_C_NUM) {
    switch(pin) {
      case USB_PLUG_DETECT_PIN:
        sensors_changed(&usb_plug_detect);
        break;
      default:
        return;
    }
  }
}


/**
 * \brief Init function for the user button.
 *
 * Parameters are ignored. They have been included because the prototype is
 * dictated by the core sensor api. The return value is also not required by
 * the API but otherwise ignored.
 *
 * \param type ignored
 * \param value ignored
 * \return ignored
 */
static int
config_user(int type, int value)
{
  config(BUTTON_USER_PORT_BASE, BUTTON_USER_PIN_MASK);

  // Disable the general gpio interrupt.
  // We enable only power-up interrupt (can equally be used while the SoC is awake)
  //GPIO_DISABLE_INTERRUPT(BUTTON_USER_PORT_BASE, BUTTON_USER_PIN_MASK);

  /* Allow the button to exit the SoC from sleep */
  //GPIO_POWER_UP_ON_RISING(BUTTON_USER_PORT, BUTTON_USER_PIN_MASK);
  GPIO_POWER_UP_ON_FALLING(BUTTON_USER_PORT, BUTTON_USER_PIN_MASK);
  GPIO_ENABLE_POWER_UP_INTERRUPT(BUTTON_USER_PORT, BUTTON_USER_PIN_MASK);

  //ioc_set_over(BUTTON_USER_PORT, BUTTON_USER_PIN, IOC_OVERRIDE_PUE);

  nvic_interrupt_enable(BUTTON_USER_VECTOR);

  gpio_register_callback(btn_callback, BUTTON_USER_PORT, BUTTON_USER_PIN);
  return 0;
}

static int
config_usbplug(int type, int value)
{

  config(USB_PLUG_DETECT_PORT_BASE, USB_PLUG_DETECT_PIN_MASK);

  // Disable the general gpio interrupt.
  // We enable only power-up interrupt (can equally be used while the SoC is awake)
  // myFIXME something is different here than the user button.
  //GPIO_DISABLE_INTERRUPT(USB_PLUG_DETECT_PORT_BASE, USB_PLUG_DETECT_PIN_MASK);
  GPIO_TRIGGER_BOTH_EDGES(USB_PLUG_DETECT_PORT_BASE, USB_PLUG_DETECT_PIN_MASK);

  /* Allow the usb plug to exit the SoC from sleep */
  GPIO_POWER_UP_ON_RISING(USB_PLUG_DETECT_PORT, USB_PLUG_DETECT_PIN_MASK);
  GPIO_ENABLE_POWER_UP_INTERRUPT(USB_PLUG_DETECT_PORT, USB_PLUG_DETECT_PIN_MASK);

  ioc_set_over(USB_PLUG_DETECT_PORT, USB_PLUG_DETECT_PIN, IOC_OVERRIDE_PDE);

  nvic_interrupt_enable(USB_PLUG_DETECT_VECTOR);

  gpio_register_callback(btn_callback, USB_PLUG_DETECT_PORT, USB_PLUG_DETECT_PIN);

  return 0;
}


void
button_sensor_init()
{
  timer_set(&debouncetimer, 0);
}


SENSORS_SENSOR(button_user_sensor, "Button User", NULL, config_user, NULL);
SENSORS_SENSOR(usb_plug_detect, "USB Plug Detect", NULL, config_usbplug, NULL);

/** @} */
