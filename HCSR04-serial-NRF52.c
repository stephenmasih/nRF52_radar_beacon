#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_serial.h"
#include "app_timer.h"


#include "app_error.h"
#include "app_util.h"
#include "boards.h"



 #define PIN_IN BSP_BUTTON_0
 #define PIN_OUT BSP_LED_0

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      SER_APP_RX_PIN, SER_APP_TX_PIN,
                      SER_APP_RTS_PIN, SER_APP_CTS_PIN,
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, sleep_handler);


NRF_SERIAL_UART_DEF(serial_uart, 0);


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
bool getDistance(float* dist)
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
  
  // wait till Echo pin goes high
  while(!nrf_gpio_pin_read(pinEcho));
  // reset counter
  tCount = 0;
    // start timer
  NRF_TIMER1->TASKS_START = 1;
  // wait till Echo pin goes low
  while(nrf_gpio_pin_read(pinEcho));
  
  // calculate duration in us
  float duration = countToUs*tCount;
 
  // dist = duration * speed of sound * 1/2
  // dist in cm = duration in us * 10^-6 * 340.29 * 100 * 1/2
  float distance = duration*0.017;
  
  // check value
  if(distance < 400.0) {

    // save
    *dist = distance;

    return true;
  }
  else {
    return false;
  }
}

// set up and start Timer1
void start_timer(void)
{		
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;  
  NRF_TIMER1->TASKS_CLEAR = 1;
  // set prescalar n
  // f = 16 MHz / 2^(n)
  uint8_t prescaler = 0;
	NRF_TIMER1->PRESCALER = prescaler; 
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;

  // 16 MHz clock generates timer tick every 1/(16000000) s = 62.5 nano s
  // With compare enabled, the interrupt is fired every: 62.5 * comp1 nano s
  // = 0.0625*comp1 micro seconds
  // multiply this by 2^(prescalar)

  uint16_t comp1 = 500;
  // set compare
	NRF_TIMER1->CC[1] = comp1;

  // set conversion factor
  countToUs = 0.0625*comp1*(1 << prescaler);

  // enable compare 1
	NRF_TIMER1->INTENSET = 
    (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);

  // use the shorts register to clear compare 1
  NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << 
                        TIMER_SHORTS_COMPARE1_CLEAR_Pos);

  // enable IRQ
  NVIC_EnableIRQ(TIMER1_IRQn);
}

// Timer 1 IRQ handler
// just increment count
void TIMER1_IRQHandler(void)
{
	if (NRF_TIMER1->EVENTS_COMPARE[1] && 
      NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk) {

    // clear compare register event	
    NRF_TIMER1->EVENTS_COMPARE[1] = 0;

    // increment count
    tCount++;
  }
}
int main(void)
{ 
    ret_code_t ret;
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);
    ret = app_timer_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);
   
    start_timer();
    // set up HC-SR04 pins
    nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
    nrf_gpio_pin_dir_set(PIN_OUT, NRF_GPIO_PIN_DIR_OUTPUT);
    static char tx_message[] = "RADAR STARTED....\n\r";

    ret = nrf_serial_write(&serial_uart,
                           tx_message,
                           strlen(tx_message),
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
    APP_ERROR_CHECK(ret);

    while (true)
    {
         int dist=23;
      getDistance(&dist);
      static char tx_message[6] ;
      sprintf(tx_message,"%ld CM\r\n", dist);
        ret = nrf_serial_write(&serial_uart,
                           tx_message,
                           strlen(tx_message),
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
        APP_ERROR_CHECK(ret);
        (void)nrf_serial_flush(&serial_uart, 0);
         nrf_delay_ms(250);

    }
}

/** @} */
