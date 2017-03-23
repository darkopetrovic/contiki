/**
 * \addtogroup sensors
 * @{
 *
 *
 * \file
 * Implementation of a generic module controlling the platform sensors
 */

#include "contiki.h"

#include "platform-sensors.h"
#include "ina3221-sensor.h"
#include "pir-sensor.h"
#include "sht21-sensor.h"
#include "button-sensor.h"
#include "ccs811-sensor.h"
#include "bmp280-sensor.h"
#include "tsl2561-sensor.h"
#include "mic-sensor.h"
#include "cpu.h"

#include "net/netstack.h"

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
#include "dev/serial-line.h"
#include "apps/shell/shell.h"
#include "apps/serial-shell/serial-shell.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
//#include "cdc-acm.h"
#include "usb/usb-serial.h"
#include "dev/usb-regs.h" /* USB_CTRL */

#if APPS_APPCONFIG
#include "shell-config.h"
#endif

#endif

#include <string.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
extern unsigned long _nrbss;
extern unsigned long _enrbss;
extern unsigned long _data;
extern unsigned long _data_lma;
extern unsigned long _nrdata;
extern unsigned long _nrdata_lma;
extern unsigned long _enrdata;
extern unsigned long _nrdata_size;

extern struct process usb_serial_process;
extern struct process usb_process;
extern struct process shell_process;
extern struct process shell_server_process;
extern struct process serial_shell_process;
#endif

#if ENERGEST_CONF_ON
static unsigned long irq_energest = 0;

#define ENERGEST_IRQ_SAVE(a) do { \
    a = energest_type_time(ENERGEST_TYPE_IRQ); } while(0)
#define ENERGEST_IRQ_RESTORE(a) do { \
    energest_type_set(ENERGEST_TYPE_IRQ, a); } while(0)
#else
#define ENERGEST_IRQ_SAVE(a) do {} while(0)
#define ENERGEST_IRQ_RESTORE(a) do {} while(0)

#endif

uint8_t reading_voltage;

void
deep_sleep_ms(uint32_t duration, int8_t port, uint8_t interrupt_pin)
{
  uint32_t i;
  uint32_t gpio_pi_en;
  uint32_t start;
  int32_t duration_left;
#if RDC_SLEEPING_HOST
  uint8_t rdc_status = 0;
#endif

  /* Prevent entering PM2 mode if USB is plugged. */
  if( USB_IS_PLUGGED() && !reading_voltage){
    for(i=0;i<duration;i++){
      clock_delay_usec(1000);
    }
  } else {

    INTERRUPTS_DISABLE();

#if RDC_SLEEPING_HOST
    rdc_status = crdc_get_rdc_status();
    if(rdc_status){
      /* Ensure that the SoC sleeps during the whole duration and won't
       * be wake-up by the RDC. */
      crdc_disable_rdc(0);
    }
#endif

    /* Store and disable temporarily gpio power-up interrupts.
     * Writing to one register change the others. Thus we clear only
     * the register for port A. */
    gpio_pi_en = REG(GPIO_PORT_TO_BASE(0) + GPIO_PI_IEN);
    REG(GPIO_PORT_TO_BASE(0) + GPIO_PI_IEN) = 0;

    /*  Enable only desired gpio power-up interrupts. */
    if(port >= 0){
      GPIO_POWER_UP_ON_FALLING(port, GPIO_PIN_MASK(interrupt_pin));
      GPIO_POWER_UP_ON_RISING(port, GPIO_PIN_MASK(interrupt_pin));
      GPIO_ENABLE_POWER_UP_INTERRUPT(port, GPIO_PIN_MASK(interrupt_pin));
      nvic_interrupt_enable(port);
    }

    duration_left = (int32_t)(((float)(duration)/1000.0)*RTIMER_SECOND);

    INTERRUPTS_ENABLE();

    // ensure that the SoC sleeps the complete duration
    do {
      start = RTIMER_NOW();
      rtimer_arch_schedule(start+duration_left);
      REG(SYS_CTRL_PMCTL) = SYS_CTRL_PMCTL_PM2;

      ENERGEST_IRQ_RESTORE(irq_energest);
      ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);
      ENTER_SLEEP_MODE();
      ENERGEST_IRQ_SAVE(irq_energest);

      // sleep timer overflow!
      if(start >= RTIMER_NOW()){
        duration_left -= 0xFFFFFFFF - start - RTIMER_NOW();
      } else {
        duration_left -= RTIMER_NOW() - start;
      }

    } while(duration_left>0);

    /* Restore gpio power-up interrupts. */
    REG(GPIO_PORT_TO_BASE(0) + GPIO_PI_IEN) = gpio_pi_en;

    if(port >= 0){
      nvic_interrupt_disable(port);
    }

#if RDC_SLEEPING_HOST
    // enable only if previously enabled
    if(rdc_status){
      crdc_enable_rdc();
    }
#else
    NETSTACK_RDC.on();
#endif
  }
}

/* Battery voltage in mV */
uint16_t
get_battery_voltage(void)
{
  uint16_t level;
  //USB_REG_DISABLE();
  deep_sleep_ms(125, NO_GPIO_INTERRUPT, 0);
  SENSORS_ACTIVATE(ina3221_sensor);
  ina3221_sensor.configure(SENSORS_DO_MEASURE, CH2|VBUS);
  level = ina3221_sensor.value(INA3221_CH2_BUS_VOLTAGE);
  SENSORS_DEACTIVATE(ina3221_sensor);
  //USB_REG_ENABLE();
  return level;
}

#if SHELL && USB_SERIAL_CONF_ENABLE && USB_SHELL_IN_NRMEM
void
zero_fill_nrbss(void)
{
  //PRINTF("_nrbss=%p _enrbss=%p\n", &_nrbss, &_enrbss);

   /* Zero-fill the nrbss segment. */
  __asm("    ldr     r0, =_nrbss\n"
      "    ldr     r1, =_enrbss\n"
      "    mov     r2, #0\n"
      "    .thumb_func\n"
      "zero_loop2:\n"
      "        cmp     r0, r1\n"
      "        it      lt\n"
      "        strlt   r2, [r0], #4\n" "        blt     zero_loop2");
}

void
copy_nrdata(void)
{
  unsigned long *pul_src, *pul_dst;
  //PRINTF("_data=%p _data_lma=%p\n", &_data, &_data_lma);
  //PRINTF("_nrdata=%p _nrdata_lma=%p\n", &_nrdata, &_nrdata_lma);
  pul_src = &_nrdata_lma;
  for(pul_dst = &_nrdata; pul_dst < &_enrdata;) {
    *pul_dst++ = *pul_src++;
  }
}

uint8_t
usb_shell_init(void)
{
  if(USB_IS_PLUGGED()){
    PRINTF("CTRL: USB Plugged.\n");
    leds_on(LEDS_YELLOW);

    copy_nrdata();
    zero_fill_nrbss();

    usb_serial_init();
    usb_serial_set_input(serial_line_input_byte);

    /* Since the commands are stored in the non-retention memory, we need
     * to re-initialize the memory every time. */
    serial_shell_init();

    shell_ping_init();
    //shell_power_init();
    shell_ps_init();
#if APPS_APPCONFIG
    shell_config_init();
#endif
    shell_ifconfig_init();
    //shell_stackusage_init();
    shell_file_init();
    shell_coffee_init();

    return 1;

  } else {
    PRINTF("CTRL: USB Unplugged.\n");
    /* Note that the PRINTF function in exit_process() is moved after the
     * process is tested to exist. Otherwise the code crashes here if the
     * process doesn't exist. */

    process_exit(&shell_process);
    process_exit(&shell_server_process);
    process_exit(&serial_shell_process);
    process_exit(&usb_serial_process);
    process_exit(&usb_process);

    // turn-off usb-module and pll
    REG(USB_CTRL) = 0;

    // clear PC0 pin
    GPIO_CLR_PIN(GPIO_PORT_TO_BASE(USB_PULLUP_PORT), GPIO_PIN_MASK(USB_PULLUP_PIN));

    return 0;
  }

}
#endif /* SHELL */

/** \brief Exports a global symbol to be used by the sensor API */
SENSORS(&button_user_sensor, &usb_plug_detect, &pir_sensor,
    &ina3221_sensor, &sht21_sensor, &ccs811_sensor, &bmp280_sensor,
    &tsl2561_sensor, &mic_sensor);

/**
 * @}
 */
