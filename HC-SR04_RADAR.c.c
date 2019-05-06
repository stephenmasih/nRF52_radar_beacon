#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_uart.h"

#include "app_error.h"
#include "app_util.h"
#include "boards.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif





#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

void uart_transfer(char ch)
{
  app_uart_put(ch);
}


void UART_config()
{
  uint32_t err_code;

    

    const app_uart_comm_params_t comm_params =
      {
          SER_APP_RX_PIN,
          SER_APP_TX_PIN,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
}






 #define PIN_IN BSP_BUTTON_0
 #define PIN_OUT BSP_LED_0


#define NRFX_TIMER1_ENABLED 1

#include "bsp.h"
#include "app_pwm.h"


uint8_t servo_pos_max = 20;
uint8_t servo_pos_min = 5;
uint8_t servo_pos_angle_45 = 6;
uint8_t servo_pos_angle_90 = 9;
uint8_t servo_pos_angle_135 = 11;


APP_PWM_INSTANCE(PWM1,0);                   // Create the instance "PWM1" using TIMER1.

void servo_config()
{
  ret_code_t err_code;
    uint8_t SERVO_PIN = ARDUINO_8_PIN;

    /* 1-channel PWM, 50Hz, output on DK LED pins, 20ms period */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(20000L, SERVO_PIN);

    /* Switch the polarity of the first channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,NULL);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
}

















// creating TIMER 4 instance for HCSR-04
const nrf_drv_timer_t TIMER_HCSR04 = NRF_DRV_TIMER_INSTANCE(4);


int distance=0;
static char tx_message[50] ;

// counter
static volatile uint32_t tCount = 0;

// HC-SR04 Trigger - P1.03
uint32_t pinTrig = ARDUINO_2_PIN;
// HC-SR04 Echo - P1.04
uint32_t pinEcho = ARDUINO_3_PIN;

// count to us (micro seconds) conversion factor
// set in start_timer()
static volatile float countToUs = 1;

// get distance measurement from HC-SR04:
// Send a 10us HIGH pulse on the Trigger pin.
// The sensor sends out a “sonic burst” of 8 cycles.
// Listen to the Echo pin, and the duration of the next HIGH 
// signal will give you the time taken by the sound to go back 
// and forth from sensor to target.
// returns true only if a valid distance is obtained
bool getDistance()
{
  // send 12us trigger pulse
  //    _
  // __| |__
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);
  nrf_gpio_pin_set(pinTrig);
  nrf_delay_ms(12);
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);

  // listen for echo and time it
  //       ____________
  // _____|            |___

  // reset counter
   tCount = 0;
   // wait till Echo pin goes high or counter is too high
   while(!nrf_gpio_pin_read(pinEcho) && tCount < 1000);
   // reset counter
   tCount = 0;
   // wait till Echo pin goes low or counter is too high
   while(nrf_gpio_pin_read(pinEcho) && tCount < 1000);

   // calculate duration in us
   float duration = countToUs*tCount;

   // dist = duration * speed of sound * 1/2
   // dist in cm = duration in us * 10^-6 * 340.29 * 100 * 1/2
   
   //always declare the variable global
   int dist = duration *0.017;
  if(dist<=30)
   {
    distance=dist;
   }

}
void timer_hcsr04_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  switch(event_type) {
    case NRF_TIMER_EVENT_COMPARE0:
    tCount++;
    break;
  default:
  //Do nothing.
     break;
  }
}
// set up and start Timer1
void start_timer(void)
{
  nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
  uint32_t time_ticks;
  uint32_t err_code = NRF_SUCCESS;

  // Configure TIMER_HCSR04
  err_code = nrf_drv_timer_init(&TIMER_HCSR04, NULL,
                                timer_hcsr04_event_handler);
  APP_ERROR_CHECK(err_code);
  time_ticks = 500;

  // set conversion factor
  int prescaler = 0;
  countToUs = 0.0625*time_ticks*(1 << prescaler);

  nrf_drv_timer_extended_compare(&TIMER_HCSR04,
                                 NRF_TIMER_CC_CHANNEL0,
                                 time_ticks,
                                 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrf_drv_timer_enable(&TIMER_HCSR04);
}

void measure_angle_0()
{
  while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_min) == NRF_ERROR_BUSY);
  getDistance();
  sprintf(tx_message,"%ld CM#0 \r\n", distance);
  for(int i=0;i<=sizeof(tx_message);i++)
  {
    uart_transfer(tx_message[i]);
  }
  distance=0;
  nrf_delay_ms(500);
}

void measure_angle_45()
{
  while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_angle_45) == NRF_ERROR_BUSY);
  getDistance();
  sprintf(tx_message,"%ld CM#45 \r\n", distance);
  for(int i=0;i<=sizeof(tx_message);i++)
  {
    uart_transfer(tx_message[i]);
  }
  distance=0;
  nrf_delay_ms(500);
}

void measure_angle_90()
{
  while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_angle_90) == NRF_ERROR_BUSY);
  getDistance();
  sprintf(tx_message,"%ld CM#90\r\n", distance);
  for(int i=0;i<=sizeof(tx_message);i++)
  {
    uart_transfer(tx_message[i]);
  }
  distance=0;
  nrf_delay_ms(500);
}

void measure_angle_135()
{
  while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_angle_135) == NRF_ERROR_BUSY);
  getDistance();
  sprintf(tx_message,"%ld CM#135\r\n", distance);
  for(int i=0;i<=sizeof(tx_message);i++)
  {
    uart_transfer(tx_message[i]);
  }
  distance=0;
  nrf_delay_ms(500);
}

void measure_angle_180()
{
  while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_max) == NRF_ERROR_BUSY);
  getDistance();
  sprintf(tx_message,"%ld CM#180\r\n", distance);
  for(int i=0;i<=sizeof(tx_message);i++)
  {
    uart_transfer(tx_message[i]);
  }
  distance=0;
  nrf_delay_ms(500);
}

int main(void)
{ 
    ret_code_t ret;
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);
    ret = app_timer_init();
    APP_ERROR_CHECK(ret);
   
    start_timer();
    // set up HC-SR04 pins
    nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);

    servo_config();
    UART_config();

    while (true)
    {
      
      measure_angle_0();
      measure_angle_45();
      measure_angle_90();
      measure_angle_135();
      measure_angle_180();

    }
}