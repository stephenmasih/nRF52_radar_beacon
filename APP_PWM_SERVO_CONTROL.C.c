#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

int main(void)
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

    uint8_t servo_pos_max = 20;
    uint8_t servo_pos_min = 5;
    uint8_t servo_pos_angle_45 = 6;
    uint8_t servo_pos_angle_90 = 9;
    uint8_t servo_pos_angle_135 = 11;
    while (true)
    {
        /* Set the duty cycle - keep trying until PWM is ready... */
        
        while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_min) == NRF_ERROR_BUSY);
        nrf_delay_ms(1000);
        while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_angle_45) == NRF_ERROR_BUSY);
        nrf_delay_ms(1000);
        while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_angle_90) == NRF_ERROR_BUSY);
        nrf_delay_ms(1000);
        while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_angle_135) == NRF_ERROR_BUSY);
        nrf_delay_ms(1000);
        while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_max) == NRF_ERROR_BUSY);
        nrf_delay_ms(1000);
     }

}